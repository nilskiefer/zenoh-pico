//
// Copyright (c) 2022 ZettaScale Technology
//
// This program and the accompanying materials are made available under the
// terms of the Eclipse Public License 2.0 which is available at
// http://www.eclipse.org/legal/epl-2.0, or the Apache License, Version 2.0
// which is available at https://www.apache.org/licenses/LICENSE-2.0.
//
// SPDX-License-Identifier: EPL-2.0 OR Apache-2.0
//
// Contributors:
//   ZettaScale Zenoh Team, <zenoh@zettascale.tech>
//

#include <version.h>

#if KERNEL_VERSION_MAJOR == 2
#include <random/rand32.h>
#else
#include <zephyr/random/random.h>
#endif

#include <errno.h>
#include <stddef.h>
#include <sys/time.h>
#include <unistd.h>

#include "zenoh-pico/config.h"
#include "zenoh-pico/system/platform.h"

// Zephyr-specific socket constants for serial transport
#define _Z_SYS_NET_SOCKET_INVALID ((const struct device *)NULL)

/*------------------ Random ------------------*/
uint8_t z_random_u8(void) { return z_random_u32(); }

uint16_t z_random_u16(void) { return z_random_u32(); }

uint32_t z_random_u32(void) { return sys_rand32_get(); }

uint64_t z_random_u64(void) {
    uint64_t ret = 0;
    ret |= z_random_u32();
    ret = ret << 32;
    ret |= z_random_u32();

    return ret;
}

void z_random_fill(void *buf, size_t len) { sys_rand_get(buf, len); }

/*------------------ Memory ------------------*/
void *z_malloc(size_t size) { return k_malloc(size); }

void *z_realloc(void *ptr, size_t size) {
    // k_realloc not implemented in Zephyr
    return NULL;
}

void z_free(void *ptr) { k_free(ptr); }

#if Z_FEATURE_MULTI_THREAD == 1

#define Z_THREADS_NUM 4

#ifdef CONFIG_TEST_EXTRA_STACK_SIZE
#define Z_PTHREAD_STACK_SIZE_DEFAULT CONFIG_MAIN_STACK_SIZE + CONFIG_TEST_EXTRA_STACK_SIZE
#elif CONFIG_TEST_EXTRA_STACKSIZE
#define Z_PTHREAD_STACK_SIZE_DEFAULT CONFIG_MAIN_STACK_SIZE + CONFIG_TEST_EXTRA_STACKSIZE
#else
#define Z_PTHREAD_STACK_SIZE_DEFAULT CONFIG_MAIN_STACK_SIZE
#endif

K_THREAD_STACK_ARRAY_DEFINE(thread_stack_area, Z_THREADS_NUM, Z_PTHREAD_STACK_SIZE_DEFAULT);
static int thread_index = 0;

/*------------------ Task ------------------*/
z_result_t _z_task_init(_z_task_t *task, z_task_attr_t *attr, void *(*fun)(void *), void *arg) {
    z_task_attr_t *lattr = NULL;
    z_task_attr_t tmp;
    if (attr == NULL) {
        (void)pthread_attr_init(&tmp);
        (void)pthread_attr_setstack(&tmp, &thread_stack_area[thread_index++], Z_PTHREAD_STACK_SIZE_DEFAULT);
        lattr = &tmp;
    }

    _Z_CHECK_SYS_ERR(pthread_create(task, lattr, fun, arg));
}

z_result_t _z_task_join(_z_task_t *task) { _Z_CHECK_SYS_ERR(pthread_join(*task, NULL)); }

z_result_t _z_task_detach(_z_task_t *task) { _Z_CHECK_SYS_ERR(pthread_detach(*task)); }

z_result_t _z_task_cancel(_z_task_t *task) { _Z_CHECK_SYS_ERR(pthread_cancel(*task)); }

void _z_task_exit(void) { pthread_exit(NULL); }

void _z_task_free(_z_task_t **task) {
    _z_task_t *ptr = *task;
    z_free(ptr);
    *task = NULL;
}

/*------------------ Mutex ------------------*/
z_result_t _z_mutex_init(_z_mutex_t *m) { _Z_CHECK_SYS_ERR(pthread_mutex_init(m, NULL)); }

z_result_t _z_mutex_drop(_z_mutex_t *m) {
    if (m == NULL) {
        return 0;
    }
    _Z_CHECK_SYS_ERR(pthread_mutex_destroy(m));
}

z_result_t _z_mutex_lock(_z_mutex_t *m) { _Z_CHECK_SYS_ERR(pthread_mutex_lock(m)); }

z_result_t _z_mutex_try_lock(_z_mutex_t *m) { _Z_CHECK_SYS_ERR(pthread_mutex_trylock(m)); }

z_result_t _z_mutex_unlock(_z_mutex_t *m) { _Z_CHECK_SYS_ERR(pthread_mutex_unlock(m)); }

z_result_t _z_mutex_rec_init(_z_mutex_rec_t *m) {
    pthread_mutexattr_t attr;
    _Z_RETURN_IF_SYS_ERR(pthread_mutexattr_init(&attr));
    _Z_RETURN_IF_SYS_ERR(pthread_mutexattr_settype(&attr, PTHREAD_MUTEX_RECURSIVE));
    _Z_RETURN_IF_SYS_ERR(pthread_mutex_init(m, &attr));
    _Z_CHECK_SYS_ERR(pthread_mutexattr_destroy(&attr));
}

z_result_t _z_mutex_rec_drop(_z_mutex_rec_t *m) {
    if (m == NULL) {
        return 0;
    }
    _Z_CHECK_SYS_ERR(pthread_mutex_destroy(m));
}

z_result_t _z_mutex_rec_lock(_z_mutex_rec_t *m) { _Z_CHECK_SYS_ERR(pthread_mutex_lock(m)); }

z_result_t _z_mutex_rec_try_lock(_z_mutex_rec_t *m) { _Z_CHECK_SYS_ERR(pthread_mutex_trylock(m)); }

z_result_t _z_mutex_rec_unlock(_z_mutex_rec_t *m) { _Z_CHECK_SYS_ERR(pthread_mutex_unlock(m)); }

/*------------------ Condvar ------------------*/
z_result_t _z_condvar_init(_z_condvar_t *cv) {
    pthread_condattr_t attr;
    pthread_condattr_init(&attr);
    pthread_condattr_setclock(&attr, CLOCK_MONOTONIC);
    _Z_CHECK_SYS_ERR(pthread_cond_init(cv, &attr));
}

z_result_t _z_condvar_drop(_z_condvar_t *cv) { _Z_CHECK_SYS_ERR(pthread_cond_destroy(cv)); }

z_result_t _z_condvar_signal(_z_condvar_t *cv) { _Z_CHECK_SYS_ERR(pthread_cond_signal(cv)); }

z_result_t _z_condvar_signal_all(_z_condvar_t *cv) { _Z_CHECK_SYS_ERR(pthread_cond_broadcast(cv)); }

z_result_t _z_condvar_wait(_z_condvar_t *cv, _z_mutex_t *m) { _Z_CHECK_SYS_ERR(pthread_cond_wait(cv, m)); }

z_result_t _z_condvar_wait_until(_z_condvar_t *cv, _z_mutex_t *m, const z_clock_t *abstime) {
    int error = pthread_cond_timedwait(cv, m, abstime);

    if (error == ETIMEDOUT) {
        return Z_ETIMEDOUT;
    }

    _Z_CHECK_SYS_ERR(error);
}
#endif  // Z_FEATURE_MULTI_THREAD == 1

/*------------------ Sleep ------------------*/
z_result_t z_sleep_us(size_t time) {
    int32_t rem = time;
    while (rem > 0) {
        rem = k_usleep(rem);  // This function is unlikely to work as expected without kernel tuning.
                              // In particular, because the lower bound on the duration of a sleep is the
                              // duration of a tick, CONFIG_SYS_CLOCK_TICKS_PER_SEC must be adjusted to
                              // achieve the resolution desired. The implications of doing this must be
                              // understood before attempting to use k_usleep(). Use with caution.
                              // From: https://docs.zephyrproject.org/apidoc/latest/group__thread__apis.html
    }

    return 0;
}

z_result_t z_sleep_ms(size_t time) {
    int32_t rem = time;
    while (rem > 0) {
        rem = k_msleep(rem);
    }

    return 0;
}

z_result_t z_sleep_s(size_t time) {
    int32_t rem = time;
    while (rem > 0) {
        rem = k_sleep(K_SECONDS(rem));
    }

    return 0;
}

/*------------------ Instant ------------------*/
z_clock_t z_clock_now(void) {
    z_clock_t now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    return now;
}

unsigned long z_clock_elapsed_us(z_clock_t *instant) {
    z_clock_t now;
    clock_gettime(CLOCK_MONOTONIC, &now);

    unsigned long elapsed = (1000000 * (now.tv_sec - instant->tv_sec) + (now.tv_nsec - instant->tv_nsec) / 1000);
    return elapsed;
}

unsigned long z_clock_elapsed_ms(z_clock_t *instant) {
    z_clock_t now;
    clock_gettime(CLOCK_MONOTONIC, &now);

    unsigned long elapsed = (1000 * (now.tv_sec - instant->tv_sec) + (now.tv_nsec - instant->tv_nsec) / 1000000);
    return elapsed;
}

unsigned long z_clock_elapsed_s(z_clock_t *instant) {
    z_clock_t now;
    clock_gettime(CLOCK_MONOTONIC, &now);

    unsigned long elapsed = now.tv_sec - instant->tv_sec;
    return elapsed;
}

void z_clock_advance_us(z_clock_t *clock, unsigned long duration) {
    clock->tv_sec += duration / 1000000;
    clock->tv_nsec += (duration % 1000000) * 1000;

    if (clock->tv_nsec >= 1000000000) {
        clock->tv_sec += 1;
        clock->tv_nsec -= 1000000000;
    }
}

void z_clock_advance_ms(z_clock_t *clock, unsigned long duration) {
    clock->tv_sec += duration / 1000;
    clock->tv_nsec += (duration % 1000) * 1000000;

    if (clock->tv_nsec >= 1000000000) {
        clock->tv_sec += 1;
        clock->tv_nsec -= 1000000000;
    }
}

void z_clock_advance_s(z_clock_t *clock, unsigned long duration) { clock->tv_sec += duration; }

/*------------------ Time ------------------*/
z_time_t z_time_now(void) {
    z_time_t now;
    gettimeofday(&now, NULL);
    return now;
}

const char *z_time_now_as_str(char *const buf, unsigned long buflen) {
    z_time_t tv = z_time_now();
    struct tm ts;
    ts = *localtime(&tv.tv_sec);
    strftime(buf, buflen, "%Y-%m-%dT%H:%M:%SZ", &ts);
    return buf;
}

unsigned long z_time_elapsed_us(z_time_t *time) {
    z_time_t now;
    gettimeofday(&now, NULL);

    unsigned long elapsed = (1000000 * (now.tv_sec - time->tv_sec) + (now.tv_usec - time->tv_usec));
    return elapsed;
}

unsigned long z_time_elapsed_ms(z_time_t *time) {
    z_time_t now;
    gettimeofday(&now, NULL);

    unsigned long elapsed = (1000 * (now.tv_sec - time->tv_sec) + (now.tv_usec - time->tv_usec) / 1000);
    return elapsed;
}

unsigned long z_time_elapsed_s(z_time_t *time) {
    z_time_t now;
    gettimeofday(&now, NULL);

    unsigned long elapsed = now.tv_sec - time->tv_sec;
    return elapsed;
}

z_result_t _z_get_time_since_epoch(_z_time_since_epoch *t) {
    z_time_t now;
    gettimeofday(&now, NULL);
    t->secs = now.tv_sec;
    t->nanos = now.tv_usec * 1000;
    return 0;
}

/*------------------ Serial ------------------*/
#if Z_FEATURE_LINK_SERIAL == 1

#include <zephyr/console/console.h>
#include <zephyr/drivers/uart.h>

// In Zephyr, we use the console device for serial communication
static const struct device *console_dev = NULL;

// Initialize console device reference
static void _z_init_console_dev(void) {
    if (console_dev == NULL) {
        console_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
    }
}

size_t _z_read_serial_internal(const _z_sys_net_socket_t sock, uint8_t *header, uint8_t *ptr, size_t len) {
    (void)sock;  // Console device doesn't use socket parameter
    _z_init_console_dev();

    if (console_dev == NULL || !device_is_ready(console_dev)) {
        return SIZE_MAX;
    }

    // For header+data protocol, read header first if needed
    if (header != NULL) {
        if (uart_poll_in(console_dev, header) < 0) {
            return SIZE_MAX;
        }
    }

    // Read data
    for (size_t i = 0; i < len; i++) {
        if (uart_poll_in(console_dev, &ptr[i]) < 0) {
            return i;  // Return partial read count
        }
    }

    return len;
}

size_t _z_send_serial_internal(const _z_sys_net_socket_t sock, uint8_t header, const uint8_t *ptr, size_t len) {
    (void)sock;  // Console device doesn't use socket parameter
    _z_init_console_dev();

    if (console_dev == NULL || !device_is_ready(console_dev)) {
        return SIZE_MAX;
    }

    // Send header if non-zero
    if (header != 0) {
        uart_poll_out(console_dev, header);
    }

    // Send data
    for (size_t i = 0; i < len; i++) {
        uart_poll_out(console_dev, ptr[i]);
    }

    return len;
}

z_result_t _z_open_serial_from_pins(_z_sys_net_socket_t *sock, uint32_t txpin, uint32_t rxpin, uint32_t baudrate) {
    // In Zephyr, serial is already configured via devicetree - pins are ignored
    (void)txpin;
    (void)rxpin;
    (void)baudrate;

    _z_init_console_dev();

    if (console_dev == NULL || !device_is_ready(console_dev)) {
        sock->_serial = _Z_SYS_NET_SOCKET_INVALID;
        return _Z_ERR_GENERIC;
    } else {
        sock->_serial = console_dev;
        return _Z_RES_OK;
    }
}

z_result_t _z_open_serial_from_dev(_z_sys_net_socket_t *sock, char *dev, uint32_t baudrate) {
    // In Zephyr, we use the devicetree console, ignore device name and baudrate
    (void)dev;
    (void)baudrate;

    return _z_open_serial_from_pins(sock, 0, 0, baudrate);
}

z_result_t _z_listen_serial_from_pins(_z_sys_net_socket_t *sock, uint32_t txpin, uint32_t rxpin, uint32_t baudrate) {
    // For serial, listen is the same as open in Zephyr
    return _z_open_serial_from_pins(sock, txpin, rxpin, baudrate);
}

z_result_t _z_listen_serial_from_dev(_z_sys_net_socket_t *sock, char *dev, uint32_t baudrate) {
    // For serial, listen is the same as open in Zephyr
    return _z_open_serial_from_dev(sock, dev, baudrate);
}

void _z_close_serial(_z_sys_net_socket_t *sock) {
    // In Zephyr, console device is always available, nothing to close
    (void)sock;
}

#endif  // Z_FEATURE_LINK_SERIAL == 1

/*------------------ Network Stubs ------------------*/
// These are stub implementations for functions that shouldn't be called
// when networking features are disabled, but are referenced by core code

#if Z_FEATURE_LINK_TCP == 0
// TCP stubs - should not be called when TCP is disabled
bool _z_endpoint_tcp_valid(const char *endpoint) {
    (void)endpoint;
    return false;
}

_z_sys_net_socket_t _z_new_link_tcp(const char *endpoint) {
    (void)endpoint;
    _z_sys_net_socket_t sock;
    sock._serial = _Z_SYS_NET_SOCKET_INVALID;
    return sock;
}
#endif

#if Z_FEATURE_RAWETH_TRANSPORT == 0
// Raw Ethernet stubs - should not be called when raweth is disabled
bool _z_endpoint_raweth_valid(const char *endpoint) {
    (void)endpoint;
    return false;
}

_z_sys_net_socket_t _z_new_link_raweth(const char *endpoint) {
    (void)endpoint;
    _z_sys_net_socket_t sock;
    sock._serial = _Z_SYS_NET_SOCKET_INVALID;
    return sock;
}

z_result_t _z_raweth_send_n_msg(const _z_sys_net_socket_t sock, const void *msgs, size_t msg_count) {
    (void)sock;
    (void)msgs;
    (void)msg_count;
    return _Z_ERR_TRANSPORT_NOT_AVAILABLE;
}
#endif

// Multicast transport stubs - always available to prevent linker errors
z_result_t _z_multicast_open_client(void *transport, const char *locator) {
    (void)transport;
    (void)locator;
    return _Z_ERR_TRANSPORT_NOT_AVAILABLE;
}

z_result_t _z_multicast_open_peer(void *transport, const char *locator) {
    (void)transport;
    (void)locator;
    return _Z_ERR_TRANSPORT_NOT_AVAILABLE;
}

z_result_t _z_multicast_transport_create(void *transport, const char *locator) {
    (void)transport;
    (void)locator;
    return _Z_ERR_TRANSPORT_NOT_AVAILABLE;
}

z_result_t _z_multicast_send_close(void *transport) {
    (void)transport;
    return _Z_ERR_TRANSPORT_NOT_AVAILABLE;
}

z_result_t _z_multicast_transport_clear(void *transport) {
    (void)transport;
    return _Z_ERR_TRANSPORT_NOT_AVAILABLE;
}

// Socket close stub - for platforms that don't use real sockets
void _z_socket_close(_z_sys_net_socket_t *sock) {
    (void)sock;
    // In Zephyr with serial transport, nothing to close
}
