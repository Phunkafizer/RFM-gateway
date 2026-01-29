#pragma once

#ifdef DEBUG
#define SDBG(x) Serial.print(x)
#define SDBGLN(...) Serial.println(__VA_ARGS__)
#else
#define SDBG(x) void()
#define SDBGLN(x) void()
#endif