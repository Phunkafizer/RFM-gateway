"""The RFM Gateway integration."""

from dataclasses import dataclass

from homeassistant.config_entries import ConfigEntry
from homeassistant.const import Platform
from homeassistant.core import HomeAssistant
from homeassistant.exceptions import ConfigEntryNotReady

from .client import (
    RfmCapabilities,
    RfmGatewayClient,
    RfmGatewayConnectionError,
    RfmGatewayProtocolError,
)
from .const import CONF_HOST, CONF_PORT, DEFAULT_PORT_HTTP, DOMAIN

__all__ = [
    "CONF_HOST",
    "DOMAIN",
    "RfmCapabilities",
    "RfmGatewayClient",
    "RfmGatewayConnectionError",
    "RfmGatewayProtocolError",
]

PLATFORMS: list[Platform] = [Platform.RADIO_FREQUENCY]


@dataclass(slots=True)
class RuntimeData:
    """Runtime data for an RFM Gateway config entry."""

    client: RfmGatewayClient
    capabilities: RfmCapabilities


async def async_setup_entry(hass: HomeAssistant, entry: ConfigEntry) -> bool:
    """Set up RFM Gateway from a config entry."""
    host = entry.data[CONF_HOST]
    port = int(entry.data.get(CONF_PORT, DEFAULT_PORT_HTTP))
    base_url = _build_base_url(host, port)
    client = RfmGatewayClient(hass=hass, base_url=base_url)

    try:
        capabilities = await client.async_get_capabilities()
    except (RfmGatewayConnectionError, RfmGatewayProtocolError) as err:
        raise ConfigEntryNotReady(
            f"Could not initialize RFM Gateway at {base_url}: {err}"
        ) from err

    entry.runtime_data = RuntimeData(client=client, capabilities=capabilities)
    await hass.config_entries.async_forward_entry_setups(entry, PLATFORMS)
    return True


async def async_unload_entry(hass: HomeAssistant, entry: ConfigEntry) -> bool:
    """Unload an RFM Gateway config entry."""
    return await hass.config_entries.async_unload_platforms(entry, PLATFORMS)


def _build_base_url(host: str, port: int = DEFAULT_PORT_HTTP) -> str:
    """Build the gateway base URL from host and port."""
    if ":" in host and not host.startswith("["):
        return f"http://[{host}]:{port}"
    return f"http://{host}:{port}"
