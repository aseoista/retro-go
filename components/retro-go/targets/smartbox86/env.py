IDF_TARGET = "esp32p4"
FW_FORMAT = "none"

# RISC-V binaries are bigger than the ESP32 ones the default partition sizes were picked for,
# and the board has 32MB of flash: give every emulator 2MB (~11MB in total).
for app in ["retro-core", "prboom-go", "gwenesis", "fmsx", "gbsp"]:
    PROJECT_APPS[app][2] = 2 * 1024 * 1024
