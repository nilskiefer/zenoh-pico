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

#include "zenoh-pico/link/config/serial.h"

#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#include "zenoh-pico/config.h"
#include "zenoh-pico/link/manager.h"
#include "zenoh-pico/system/common/serial.h"
#include "zenoh-pico/system/link/serial.h"
#include "zenoh-pico/utils/pointers.h"

#if Z_FEATURE_LINK_SERIAL == 1

#define SPP_MAXIMUM_PAYLOAD 128

z_result_t _z_endpoint_serial_valid(_z_endpoint_t *endpoint) {
    z_result_t ret = _Z_RES_OK;

    _z_string_t ser_str = _z_string_alias_str(SERIAL_SCHEMA);
    if (!_z_string_equals(&endpoint->_locator._protocol, &ser_str)) {
        ret = _Z_ERR_CONFIG_LOCATOR_INVALID;
    }

    if (ret == _Z_RES_OK) {
        size_t addr_len = _z_string_len(&endpoint->_locator._address);
        const char *p_start = _z_string_data(&endpoint->_locator._address);
        const char *p_dot = (char *)memchr(p_start, (int)'.', addr_len);
        if (p_dot != NULL) {
            size_t dot_loc = _z_ptr_char_diff(p_dot, p_start);
            // Check if dot is first or last character
            if ((dot_loc == 0) || (dot_loc == addr_len)) {
                ret = _Z_ERR_CONFIG_LOCATOR_INVALID;
            }
        } else {
            if (_z_string_len(&endpoint->_locator._address) == (size_t)0) {
                ret = _Z_ERR_CONFIG_LOCATOR_INVALID;
            }
        }
    }
    return ret;
}

static char *__z_convert_address_serial(_z_string_t *address) {
    char *ret = NULL;
    ret = (char *)z_malloc(_z_string_len(address) + 1);
    if (ret != NULL) {
        _z_str_n_copy(ret, _z_string_data(address), _z_string_len(address) + 1);
    }
    return ret;
}

z_result_t _z_f_link_open_serial(_z_link_t *zl) {
    printf("DEBUG: _z_f_link_open_serial called\n");
    z_result_t ret = _Z_RES_OK;

    // Parse the endpoint to get the device and baudrate
    const char *address = _z_string_data(&zl->_endpoint._locator._address);
    printf("DEBUG: address = %s\n", address);
    uint32_t baudrate = 115200;  // Default baudrate

    // Check if there's a baudrate parameter
    char *baudrate_str = _z_str_intmap_get(&zl->_endpoint._config, SERIAL_CONFIG_BAUDRATE_KEY);
    if (baudrate_str != NULL) {
        baudrate = (uint32_t)strtoul(baudrate_str, NULL, 10);
    }
    printf("DEBUG: baudrate = %u\n", baudrate);

    // For console endpoints, use the console device
    if (strcmp(address, "console") == 0) {
        printf("DEBUG: Opening console device\n");
        ret = _z_open_serial_from_dev(&zl->_socket._serial._sock, NULL, baudrate);
    } else {
        printf("DEBUG: Opening device: %s\n", address);
        // For other devices, use the address as device path
        ret = _z_open_serial_from_dev(&zl->_socket._serial._sock, (char *)address, baudrate);
    }

    printf("DEBUG: _z_open_serial_from_dev returned %d, socket = %p\n", ret, zl->_socket._serial._sock);

    return ret;
}

z_result_t _z_f_link_listen_serial(_z_link_t *zl) {
    z_result_t ret = _Z_RES_OK;

    // Parse the endpoint to get the device and baudrate
    const char *address = _z_string_data(&zl->_endpoint._locator._address);
    uint32_t baudrate = 115200;  // Default baudrate

    // Check if there's a baudrate parameter
    char *baudrate_str = _z_str_intmap_get(&zl->_endpoint._config, SERIAL_CONFIG_BAUDRATE_KEY);
    if (baudrate_str != NULL) {
        baudrate = (uint32_t)strtoul(baudrate_str, NULL, 10);
    }

    // For console endpoints, use the console device
    if (strcmp(address, "console") == 0) {
        ret = _z_listen_serial_from_dev(&zl->_socket._serial._sock, NULL, baudrate);
    } else {
        // For other devices, use the address as device path
        ret = _z_listen_serial_from_dev(&zl->_socket._serial._sock, (char *)address, baudrate);
    }

    return ret;
}

void _z_f_link_close_serial(_z_link_t *zl) { _z_close_serial(&zl->_socket._serial._sock); }

void _z_f_link_free_serial(_z_link_t *zl) {
    // No specific cleanup needed for serial
    _ZP_UNUSED(zl);
}

size_t _z_f_link_write_serial(const _z_link_t *zl, const uint8_t *ptr, size_t len, _z_sys_net_socket_t *socket) {
    if (socket != NULL) {
        return _z_send_serial_internal(*socket, 0, ptr, len);
    } else {
        return _z_send_serial_internal(zl->_socket._serial._sock, 0, ptr, len);
    }
}

size_t _z_f_link_write_all_serial(const _z_link_t *zl, const uint8_t *ptr, size_t len) {
    return _z_send_serial_internal(zl->_socket._serial._sock, 0, ptr, len);
}

size_t _z_f_link_read_serial(const _z_link_t *zl, uint8_t *ptr, size_t len, _z_slice_t *addr) {
    _ZP_UNUSED(addr);
    uint8_t header;
    return _z_read_serial_internal(zl->_socket._serial._sock, &header, ptr, len);
}

size_t _z_f_link_read_exact_serial(const _z_link_t *zl, uint8_t *ptr, size_t len, _z_slice_t *addr,
                                   _z_sys_net_socket_t *socket) {
    _ZP_UNUSED(addr);
    if (socket != NULL) {
        return _z_read_exact_serial(*socket, ptr, len);
    } else {
        return _z_read_exact_serial(zl->_socket._serial._sock, ptr, len);
    }
}

size_t _z_f_link_read_socket_serial(const _z_sys_net_socket_t socket, uint8_t *ptr, size_t len) {
    uint8_t header;
    return _z_read_serial_internal(socket, &header, ptr, len);
}

uint16_t _z_get_link_mtu_serial(void) { return _Z_SERIAL_MTU_SIZE; }

z_result_t _z_new_link_serial(_z_link_t *zl, _z_endpoint_t endpoint) {
    z_result_t ret = _Z_RES_OK;
    zl->_type = _Z_LINK_TYPE_SERIAL;
    zl->_cap._transport = Z_LINK_CAP_TRANSPORT_UNICAST;
    zl->_cap._flow = Z_LINK_CAP_FLOW_DATAGRAM;
    zl->_cap._is_reliable = false;

    zl->_mtu = _z_get_link_mtu_serial();

    zl->_endpoint = endpoint;

    zl->_open_f = _z_f_link_open_serial;
    zl->_listen_f = _z_f_link_listen_serial;
    zl->_close_f = _z_f_link_close_serial;
    zl->_free_f = _z_f_link_free_serial;

    zl->_write_f = _z_f_link_write_serial;
    zl->_write_all_f = _z_f_link_write_all_serial;
    zl->_read_f = _z_f_link_read_serial;
    zl->_read_exact_f = _z_f_link_read_exact_serial;
    zl->_read_socket_f = _z_f_link_read_socket_serial;

    return ret;
}
#endif
