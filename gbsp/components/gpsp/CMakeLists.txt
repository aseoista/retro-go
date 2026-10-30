set(COMPONENT_SRCS
    core/bios_data.S
    core/cheats.c
    core/cpu.cc
    core/cpu_threaded.c
    core/gba_cc_lut.c
    core/gba_memory.c
    core/gba_rom_scan.c
    core/gbp.c
    core/input.c
    core/main.c
    core/memmap.c
    core/rfu.c
    core/savestate.c
    core/serial.c
    core/serial_proto.c
    core/sound.c
    core/video.cc
    core/libretro/libretro.c
    core/libretro/libretro-common/compat/compat_posix_string.c
    core/libretro/libretro-common/compat/compat_strl.c
    core/libretro/libretro-common/compat/fopen_utf8.c
    core/libretro/libretro-common/encodings/encoding_utf.c
    core/libretro/libretro-common/file/file_path.c
    core/libretro/libretro-common/file/file_path_io.c
    core/libretro/libretro-common/streams/file_stream.c
    core/libretro/libretro-common/string/stdstring.c
    core/libretro/libretro-common/time/rtime.c
    core/libretro/libretro-common/vfs/vfs_implementation.c
    core/riscv/rv32_affine_pie.S
    core/riscv/rv32_arm_alu.c
    core/riscv/rv32_arm_branch.c
    core/riscv/rv32_arm_flow_alu.c
    core/riscv/rv32_arm_flow_full.c
    core/riscv/rv32_arm_io.S
    core/riscv/rv32_arm_memory.c
    core/riscv/rv32_arm_mul.c
    core/riscv/rv32_arm_palette.S
    core/riscv/rv32_arm_system.c
    core/riscv/rv32_block_emit.c
    core/riscv/rv32_block_link.c
    core/riscv/rv32_block_runtime.cc
    core/riscv/rv32_codegen.c
    core/riscv/rv32_fallback.c
    core/riscv/rv32_flow_timing.c
    core/riscv/rv32_interpreter_stub.S
    core/riscv/rv32_platform.c
    core/riscv/rv32_ram_units.c
    core/riscv/rv32_rom_load.S
    core/riscv/rv32_stub.S
    core/riscv/rv32_thumb_alu.c
    core/riscv/rv32_thumb_exit_shared.S
    core/riscv/rv32_thumb_flow_full.c
    core/riscv/rv32_thumb_memory.c
    core/riscv/rv32_thumb_multi_ram.S
    core/riscv/rv32_thumb_ram_helper.S
    core/riscv/rv32_thumb_ram_store.S
)
set(COMPONENT_ADD_INCLUDEDIRS "core/include" "core/libretro/libretro-common/include")
set(COMPONENT_PRIV_INCLUDEDIRS "core" "core/libretro")
set(COMPONENT_ADD_LDFRAGMENTS "gpsp.lf")
register_component()

# The assembler's dependency scan doesn't see .incbin, so rebuild the BIOS blob when it changes.
set_source_files_properties(core/bios_data.S PROPERTIES
    OBJECT_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/core/bios/open_gba_bios.bin)

# JIT code cache limits (KiB, PSRAM) and the ROM buffer limit (MiB) used by the frontend.
set(GPSP_RV32_ROM_CACHE_KIB 2048)
set(GPSP_RV32_RAM_CACHE_KIB 2048)
set(GPSP_RV32_CODE_CHUNK_KIB 256)

target_compile_definitions(${COMPONENT_TARGET} PRIVATE
    FRONTEND_SUPPORTS_RGB565
    HAVE_INTTYPES_H
    HAVE_STDINT_H
    HAVE_STRINGS_H
    INLINE=inline
    __LIBRETRO__
    GPSP_RV32_JIT=1
    GPSP_RV32_ROM_CACHE_KIB=${GPSP_RV32_ROM_CACHE_KIB}
    GPSP_RV32_RAM_CACHE_KIB=${GPSP_RV32_RAM_CACHE_KIB}
    GPSP_RV32_CODE_CHUNK_KIB=${GPSP_RV32_CODE_CHUNK_KIB}
    GPSP_AFFINE_PIE=1
)

# ESP32-P4 before rev 3 has no Zba/Zbb/Zbs/Zcb: build the core and generate JIT code without
# them. Rev 3 parts could enable them all for a few more percent.
# Its PIE is also the older XespV 2.1, with different encodings: "xesppie" (as IDF uses for
# these revisions) selects that assembler; "xespv" would emit 2.2 code that traps as an
# illegal PIE instruction in the affine renderer. The core uses no hardware loops.
if(CONFIG_ESP32P4_REV_MIN_FULL LESS 300)
    set(GPSP_RV32_ZB 0)
    set(GPSP_RV32_MARCH "rv32imafc_zicsr_zifencei_xesppie")
else()
    set(GPSP_RV32_ZB 1)
    set(GPSP_RV32_MARCH "rv32imafc_zicsr_zifencei_zba_zbb_zbs_zcb_xesploop_xespv")
endif()
target_compile_definitions(${COMPONENT_TARGET} PRIVATE
    GPSP_RV32_USE_RVC=1
    GPSP_RV32_USE_ZBA=${GPSP_RV32_ZB}
    GPSP_RV32_USE_ZBB=${GPSP_RV32_ZB}
    GPSP_RV32_USE_ZBS=${GPSP_RV32_ZB}
    GPSP_RV32_USE_ZCB=${GPSP_RV32_ZB}
)

target_compile_options(${COMPONENT_TARGET} PRIVATE
    -march=${GPSP_RV32_MARCH}
    $<$<COMPILE_LANGUAGE:CXX>:-fno-exceptions>
    $<$<COMPILE_LANGUAGE:CXX>:-fno-rtti>
    $<$<COMPILE_LANGUAGE:CXX>:-std=c++11>
)

rg_setup_compile_options(-O2 -Wno-format -Wno-unused-function -Wno-unused-variable -Wno-sign-compare)
