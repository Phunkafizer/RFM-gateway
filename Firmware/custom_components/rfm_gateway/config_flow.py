"""Config flow for the RFM Gateway integration."""

import ipaddress
from typing import Any

import voluptuous as vol

from homeassistant import config_entries
from homeassistant.config_entries import ConfigFlowResult
from homeassistant.helpers.service_info.zeroconf import ZeroconfServiceInfo

from .client import RfmGatewayClient, RfmGatewayConnectionError, RfmGatewayProtocolError
from .const import CONF_HOST, CONF_PORT, DEFAULT_PORT_HTTP, DOMAIN


def _discovery_property_text(value: Any) -> str:
    """Return a normalized text representation for zeroconf properties."""
    if isinstance(value, bytes):
        return value.decode(errors="ignore").strip().lower()
    if value is None:
        return ""
    return str(value).strip().lower()


class RfmGatewayConfigFlow(config_entries.ConfigFlow, domain=DOMAIN):
    """Handle a config flow for RFM Gateway."""

    VERSION = 1
    _discovered_host: str | None = None

    async def async_step_user(
        self, user_input: dict[str, Any] | None = None
    ) -> ConfigFlowResult:
        """Handle the user step."""
        errors: dict[str, str] = {}

        if user_input is not None:
            host = str(user_input[CONF_HOST]).strip()
            host = self._normalize_host(host)
            port = int(user_input.get(CONF_PORT, DEFAULT_PORT_HTTP))

            try:
                capabilities = await self._async_get_capabilities(host, port)
            except RfmGatewayConnectionError:
                errors["base"] = "cannot_connect"
            except RfmGatewayProtocolError:
                errors["base"] = "invalid_response"
            else:
                self._async_abort_entries_match({CONF_HOST: host})

                title = capabilities.device_name or f"RFM Gateway ({host})"
                return self.async_create_entry(
                    title=title,
                    data={
                        CONF_HOST: host,
                        CONF_PORT: port,
                    },
                )

        schema = vol.Schema(
            {
                vol.Required(CONF_HOST): str,
                vol.Optional(CONF_PORT, default=DEFAULT_PORT_HTTP): int,
            }
        )
        return self.async_show_form(step_id="user", data_schema=schema, errors=errors)

    async def async_step_zeroconf(
        self, discovery_info: ZeroconfServiceInfo
    ) -> ConfigFlowResult:
        """Handle a flow initialized by zeroconf discovery."""
        model = _discovery_property_text(discovery_info.properties.get("model"))
        rf_api = _discovery_property_text(discovery_info.properties.get("rf_api"))
        is_named_gateway = str(discovery_info.name).lower().startswith("rfm-gateway")
        if model != "rfm-gateway" and rf_api != "1" and not is_named_gateway:
            return self.async_abort(reason="not_rfm_gateway")

        # Zeroconf service instance names are stable identifiers for discovered devices.
        unique_id = str(discovery_info.name).rstrip(".")
        await self.async_set_unique_id(unique_id)
        self._abort_if_unique_id_configured()

        host = ""

        ip_address = getattr(discovery_info, "ip_address", None)
        if ip_address is not None:
            preferred = self._preferred_discovery_ip(str(ip_address))
            if preferred:
                host = preferred

        if not host:
            for addr in getattr(discovery_info, "ip_addresses", []) or []:
                if addr is None:
                    continue
                preferred = self._preferred_discovery_ip(str(addr))
                if preferred:
                    host = preferred
                    break

        raw_host = self._normalize_host(discovery_info.host or "")
        raw_hostname = self._normalize_host(discovery_info.hostname or "")

        if not host and self._is_ip_address(raw_host):
            preferred = self._preferred_discovery_ip(raw_host)
            if preferred:
                host = preferred

        if not host and self._is_ip_address(raw_hostname):
            preferred = self._preferred_discovery_ip(raw_hostname)
            if preferred:
                host = preferred

        if not host and self._is_usable_discovery_host(raw_host):
            host = raw_host

        if not host and self._is_usable_discovery_host(raw_hostname):
            host = raw_hostname

        if not host:
            return self.async_abort(reason="not_rfm_gateway")

        self._abort_if_unique_id_configured(updates={CONF_HOST: host})

        pretty_name = f"RFM Gateway {host}"

        self._discovered_host = host
        self.context["discovered_port"] = int(discovery_info.port or DEFAULT_PORT_HTTP)
        self.context["title_placeholders"] = {"host": host, "name": pretty_name}
        return await self.async_step_zeroconf_confirm()

    async def async_step_zeroconf_confirm(
        self, user_input: dict[str, Any] | None = None
    ) -> ConfigFlowResult:
        """Confirm setup for a discovered gateway."""
        errors: dict[str, str] = {}
        host = self._discovered_host
        port = int(self.context.get("discovered_port", DEFAULT_PORT_HTTP))
        if host is None:
            return self.async_abort(reason="unknown")

        if user_input is not None:
            try:
                capabilities = await self._async_get_capabilities(host, port)
            except RfmGatewayConnectionError:
                errors["base"] = "cannot_connect"
            except RfmGatewayProtocolError:
                errors["base"] = "invalid_response"
            else:
                title = capabilities.device_name or f"RFM Gateway ({host})"
                return self.async_create_entry(
                    title=title,
                    data={CONF_HOST: host, CONF_PORT: port},
                )

        return self.async_show_form(
            step_id="zeroconf_confirm",
            description_placeholders={"host": host, "port": str(port)},
            errors=errors,
        )

    @staticmethod
    def _build_base_url(host: str, port: int = DEFAULT_PORT_HTTP) -> str:
        if ":" in host and not host.startswith("["):
            return f"http://[{host}]:{port}"
        return f"http://{host}:{port}"

    async def _async_get_capabilities(self, host: str, port: int = DEFAULT_PORT_HTTP):
        client = RfmGatewayClient(
            hass=self.hass,
            base_url=self._build_base_url(host, port),
        )
        return await client.async_get_capabilities()

    async def _async_validate_host(self, host: str, port: int = DEFAULT_PORT_HTTP) -> None:
        await self._async_get_capabilities(host, port)

    @staticmethod
    def _format_frequency_range(ranges: list[tuple[int, int]]) -> str:
        if not ranges:
            return ""
        formatted = []
        for min_hz, max_hz in ranges:
            min_mhz = min_hz / 1_000_000
            max_mhz = max_hz / 1_000_000
            formatted.append(f"{min_mhz:.0f}-{max_mhz:.0f} MHz")
        return "Supported: " + ", ".join(formatted)

    @staticmethod
    def _normalize_host(host: str) -> str:
        result = host.strip().rstrip(".")
        if not result:
            return result
        if result.endswith(".local"):
            return result
        if result.startswith("["):
            end = result.find("]")
            if end != -1:
                return result[1:end]
            return result
        try:
            ipaddress.ip_address(result)
        except ValueError:
            pass
        else:
            return result
        if result.count(":") == 1:
            return result.rsplit(":", 1)[0]
        return result

    @staticmethod
    def _is_ip_address(value: str) -> bool:
        if not value:
            return False
        try:
            ipaddress.ip_address(value)
        except ValueError:
            return False
        else:
            return True

    @staticmethod
    def _preferred_discovery_ip(value: str) -> str | None:
        if not value:
            return None

        try:
            ip = ipaddress.ip_address(value)
        except ValueError:
            return None

        if ip.version == 4:
            return str(ip)

        if ip.is_link_local:
            return None

        return str(ip)

    @staticmethod
    def _is_usable_discovery_host(value: str) -> bool:
        if not value:
            return False
        try:
            ipaddress.ip_address(value)
        except ValueError:
            pass
        else:
            return False
        if "_http._tcp" in value:
            return False
        if "._" in value:
            return False
        if "_" in value:
            return False
        return True
