#pragma once

#include <stdbool.h>
#include <stddef.h>

// A transport moves bytes between us and a printer (Bluetooth SPP today, BLE GATT later...)

typedef struct
{
    char name[32];
    char address[18]; // "AA:BB:CC:DD:EE:FF"
    bool is_printer;  // The device advertises itself as a printer (Class of Device: imaging)
} printer_device_t;

typedef struct
{
    const char *name;
    bool (*init)(void);
    void (*deinit)(void);
    // Returns the number of devices found, or -1 on error. Optional.
    int (*scan)(printer_device_t *devices, int max_devices, int timeout_ms);
    bool (*open)(const char *address);
    void (*close)(void);
    // Blocks while the printer can't accept more data. Returns the bytes written, or -1 on error.
    int (*write)(const void *data, size_t length, int timeout_ms);
    // Returns the bytes read (0 on timeout), or -1 on error.
    int (*read)(void *buffer, size_t length, int timeout_ms);
} printer_transport_t;

extern const printer_transport_t transport_spp;
extern const printer_transport_t transport_file;
