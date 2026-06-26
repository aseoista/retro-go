set(COMPONENT_SRCS
    cpu.cc
    video.cc
    gba_memory.c
    savestate.c
    input.c
    sound.c
    cheats.c
    memmap.c
    serial.c
    gbp.c
    rfu.c
    serial_proto.c
    gba_cc_lut.c
    main_gpsp.c
    libretro/libretro.c
    libretro/libretro-common/compat/compat_posix_string.c
    libretro/libretro-common/compat/compat_strl.c
    libretro/libretro-common/compat/fopen_utf8.c
    libretro/libretro-common/encodings/encoding_utf.c
    libretro/libretro-common/file/file_path.c
    libretro/libretro-common/file/file_path_io.c
    libretro/libretro-common/streams/file_stream.c
    libretro/libretro-common/string/stdstring.c
    libretro/libretro-common/time/rtime.c
    libretro/libretro-common/vfs/vfs_implementation.c
    bios/gba_bios_normatt.c
)

set(COMPONENT_ADD_INCLUDEDIRS
    "."
    "libretro"
    "libretro/libretro-common/include"
    "bios"
)

set(COMPONENT_REQUIRES "retro-go")

register_component()

target_compile_definitions(${COMPONENT_TARGET} PRIVATE
    THREADED_RENDERER=0
    GPSP_NO_CHEATS=1
)

rg_setup_compile_options(-O3 -Wno-unused-function -Wno-sign-compare -Wno-unused-variable -fno-exceptions)
