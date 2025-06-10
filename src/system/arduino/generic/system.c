// zenoh-pico/src/system/arduino_generic/system.c

#include "sys.h"
#include "zenoh-pico/config.h"

// =================================================================================================
// TIME
// =================================================================================================
z_time_t z_time_now(void) { return micros(); }

unsigned long z_time_elapsed_us(const z_time_t *start) { return micros() - *start; }

unsigned long z_time_elapsed_ms(const z_time_t *start) { return (micros() - *start) / 1000; }

// =================================================================================================
// SLEEP
// =================================================================================================
void z_sleep_ms(uint32_t ms) { delay(ms); }

// =================================================================================================
// MEMORY
// =================================================================================================
void *z_malloc(size_t size) { return malloc(size); }

void *z_realloc(void *ptr, size_t size) { return realloc(ptr, size); }

void z_free(void *ptr) { free(ptr); }

// =================================================================================================
// RANDOM
// =================================================================================================
// Note: Arduino's random is not cryptographically secure, but sufficient for peer ID generation.
uint32_t z_random_u32(void) { return random(0, 0xFFFFFFFF); }

void z_random_fill(void *buf, size_t len) {
    uint8_t *p = (uint8_t *)buf;
    for (size_t i = 0; i < len; i++) {
        p[i] = (uint8_t)random(0, 256);
    }
}