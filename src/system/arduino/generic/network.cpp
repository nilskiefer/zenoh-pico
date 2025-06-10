// zenoh-pico/src/system/arduino_generic/network.cpp

#include "sys.h"

extern "C" {
#include "zenoh-pico/protocol/codec/serial.h"
#include "zenoh-pico/utils/logging.h"
}

// The serial implementation expects data to be framed and encoded using COBS.
// These are internal zenoh-pico functions we need to call.
size_t _z_serial_msg_serialize(uint8_t *raw_buf, size_t raw_buf_len, const uint8_t *payload, size_t payload_len,
                               uint8_t header, uint8_t *cobs_buf, size_t cobs_buf_len);
size_t _z_serial_msg_deserialize(const uint8_t *raw_buf, size_t raw_buf_len, uint8_t *payload, size_t payload_len,
                                 uint8_t *header, uint8_t *cobs_buf, size_t cobs_buf_len);

// We only implement the functions required for Z_FEATURE_LINK_SERIAL
#if Z_FEATURE_LINK_SERIAL == 1

// Opens the serial port. The `dev` parameter is ignored; we always use the default `Serial`.
z_result_t _z_open_serial_from_dev(_z_sys_net_socket_t *sock, char *dev, uint32_t baudrate) {
    (void)dev;  // Device path is ignored, we hardcode to the primary USB-Serial port.

    // Point our socket handle to the global `Serial` object
    *sock = &Serial;

    // Initialize the serial port
    (*sock)->begin(baudrate);

    // Some boards need a moment for the serial port to be ready.
    // A timeout is good practice here to avoid getting stuck forever.
    unsigned long start_time = millis();
    while (!(*sock) && (millis() - start_time < 2000)) {
        z_sleep_ms(10);
    }

    if (!(*sock)) {
        return Z_ERROR;  // Failed to open serial port
    }

    return Z_OK;
}

void _z_close_serial(_z_sys_net_socket_t *sock) {
    if (sock && *sock) {
        (*sock)->end();
    }
}

// Reads one COBS-encoded frame from the serial port.
size_t _z_read_serial_internal(const _z_sys_net_socket_t sock, uint8_t *header, uint8_t *ptr, size_t len) {
    uint8_t cobs_buf[_Z_SERIAL_MAX_COBS_BUF_SIZE];
    size_t received = 0;

    // Read until we get a COBS frame delimiter (0x00) or the buffer is full
    while (received < sizeof(cobs_buf)) {
        if (sock->available() > 0) {
            uint8_t b = sock->read();
            cobs_buf[received++] = b;
            if (b == 0) {
                break;  // End of frame
            }
        } else {
            // If there's no data, yield for a moment to avoid blocking everything.
            z_sleep_ms(1);
        }
    }

    if (received == 0) {
        return 0;  // No data read
    }

    // A temporary buffer for the deserialization process.
    uint8_t tmp_buf[_Z_SERIAL_MFS_SIZE];

    // Deserialize the COBS frame back into a zenoh message
    size_t decoded_len = _z_serial_msg_deserialize(cobs_buf, received, ptr, len, header, tmp_buf, sizeof(tmp_buf));

    // SIZE_MAX indicates a decoding error.
    if (decoded_len == SIZE_MAX) {
        return 0;
    }

    return decoded_len;
}

// Sends one zenoh message by encoding it into a COBS frame first.
size_t _z_send_serial_internal(const _z_sys_net_socket_t sock, uint8_t header, const uint8_t *ptr, size_t len) {
    uint8_t cobs_buf[_Z_SERIAL_MAX_COBS_BUF_SIZE];
    uint8_t tmp_buf[_Z_SERIAL_MFS_SIZE];

    // Serialize the zenoh message into a COBS frame
    size_t encoded_len =
        _z_serial_msg_serialize(cobs_buf, sizeof(cobs_buf), ptr, len, header, tmp_buf, sizeof(tmp_buf));

    if (encoded_len == SIZE_MAX) {
        return SIZE_MAX;  // Encoding error
    }

    // Write the encoded frame to the serial port
    size_t written = sock->write(cobs_buf, encoded_len);

    if (written != encoded_len) {
        return SIZE_MAX;  // Write error
    }

    return len;  // On success, return the original payload length
}

#endif  // Z_FEATURE_LINK_SERIAL