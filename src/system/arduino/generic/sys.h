// zenoh-pico/src/system/arduino_generic/sys.h

#ifndef Z_SYS_H
#define Z_SYS_H

#include <Arduino.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

// For a generic Arduino serial port, we can point directly to the HardwareSerial object.
// This makes our network.cpp implementation clean.
typedef HardwareSerial* _z_sys_net_socket_t;

// Endpoints are not used by the serial link, but the type must exist.
typedef void* _z_sys_net_endpoint_t;

// The time source for Arduino is the microsecond counter.
typedef unsigned long z_time_t;

#endif /* Z_SYS_H */