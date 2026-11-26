cmake_minimum_required(VERSION 3.21)
project(DartUnifiedAcquisition LANGUAGES C ASM)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_EXTENSIONS ON)
set(CMAKE_C_FLAGS_RELWITHDEBINFO "-O2 -g3")
set(CMAKE_C_FLAGS_DEBUG "-Og -g3")

set(V "${CMAKE_CURRENT_SOURCE_DIR}/vendor")
set(H "${CMAKE_CURRENT_SOURCE_DIR}/h750")
add_executable(dart_f103 f103/main.c common/protocol.c
    ${V}/cmsis-device-f1/Source/Templates/system_stm32f1xx.c
    ${V}/cmsis-device-f1/Source/Templates/gcc/startup_stm32f103xb.s)
target_include_directories(dart_f103 PRIVATE common ${V}/cmsis-device-f1/Include ${V}/CMSIS_5/Core/Include)
target_compile_definitions(dart_f103 PRIVATE STM32F103xB HSE_VALUE=12000000U)

set(H7_HAL_SOURCES)
foreach(name hal hal_cortex hal_dma hal_dma_ex hal_pcd hal_pcd_ex ll_usb hal_dcmi hal_exti
        hal_flash hal_flash_ex hal_gpio hal_pwr hal_pwr_ex hal_rcc hal_rcc_ex hal_sd hal_sd_ex
        hal_uart hal_uart_ex ll_sdmmc hal_jpeg hal_mdma)
    list(APPEND H7_HAL_SOURCES "${V}/stm32h7xx-hal-driver/Src/stm32h7xx_${name}.c")
endforeach()
add_executable(dart_h750
    h750/main.c h750/board.c h750/camera.c h750/camera_test.c h750/diskio_sd.c
    h750/sd_trace.c h750/usbd_conf.c h750/usb_tf.c common/protocol.c common/avi.c common/auto_record.c
    h750/green_detector.c h750/green_annotate.c h750/jpeg_hw.c h750/jpeg_mcu.c h750/record_profile.c
    ${V}/ov5640/ov5640.c ${V}/ov5640/ov5640_reg.c ${V}/fatfs/source/ff.c
    ${V}/cmsis-device-h7/Source/Templates/system_stm32h7xx.c
    ${V}/cmsis-device-h7/Source/Templates/gcc/startup_stm32h750xx.s
    ${H7_HAL_SOURCES}
    ${V}/usb-device/Core/Src/usbd_core.c ${V}/usb-device/Core/Src/usbd_ctlreq.c
    ${V}/usb-device/Core/Src/usbd_ioreq.c ${V}/usb-device/Class/MSC/Src/usbd_msc.c
    ${V}/usb-device/Class/MSC/Src/usbd_msc_bot.c ${V}/usb-device/Class/MSC/Src/usbd_msc_scsi.c
    ${V}/usb-device/Class/MSC/Src/usbd_msc_data.c)
target_include_directories(dart_h750 PRIVATE common h750 ${V}/ov5640
    ${V}/cmsis-device-h7/Include ${V}/stm32h7xx-hal-driver/Inc ${V}/CMSIS_5/Core/Include
    ${V}/fatfs/source ${V}/usb-device/Core/Inc ${V}/usb-device/Class/MSC/Inc)
target_compile_definitions(dart_h750 PRIVATE STM32H750xx USE_HAL_DRIVER HSE_VALUE=12000000U)
foreach(name GoIdleState OperCond AppCommand AppOperCommand SendCID SetRelAdd SendCSD
        SelDesel BlockLength StatusRegister SendSCR BusWidth SendStatus ReadSingleBlock)
    target_link_options(dart_h750 PRIVATE "-Wl,--wrap=SDMMC_Cmd${name}")
endforeach()

foreach(chip f103 h750)
    set(target "dart_${chip}")
    if(chip STREQUAL "f103")
        set(cpu cortex-m3)
        set(device STM32F103C8T6)
        set(flash_limit 65536)
        set(firmware_version "2.0.0")
    else()
        set(cpu cortex-m7)
        set(device STM32H750VBT6)
        set(flash_limit 131072)
        set(firmware_version "2.1.6-latest")
    endif()
    target_compile_options(${target} PRIVATE -mcpu=${cpu} -mthumb -mfloat-abi=soft
        -ffunction-sections -fdata-sections -fno-common -gdwarf-4 -Wall -Wextra -Werror
        -Wno-misleading-indentation
        "-ffile-prefix-map=${CMAKE_CURRENT_SOURCE_DIR}=/dart_firmware"
        "-fdebug-prefix-map=${CMAKE_CURRENT_SOURCE_DIR}=/dart_firmware")
    target_link_options(${target} PRIVATE -mcpu=${cpu} -mthumb -mfloat-abi=soft
        -nostartfiles --specs=nano.specs --specs=nosys.specs
        "-T${CMAKE_CURRENT_SOURCE_DIR}/${chip}/linker.ld" -Wl,--gc-sections
        "-Wl,-Map=${CMAKE_CURRENT_BINARY_DIR}/${chip}/${target}.map")
    target_link_libraries(${target} PRIVATE c gcc m)
    set_target_properties(${target} PROPERTIES SUFFIX ".elf"
        RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/${chip}"
        LINK_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/${chip}/linker.ld")
    add_custom_command(TARGET ${target} POST_BUILD
        COMMAND "${CMAKE_OBJCOPY}" -O binary "${chip}/${target}.elf" "${chip}/${target}.bin"
        COMMAND "${CMAKE_OBJCOPY}" -O ihex "${chip}/${target}.elf" "${chip}/${target}.hex"
        COMMAND "${DART_ARM_SIZE}" "${chip}/${target}.elf"
        COMMAND "${CMAKE_COMMAND}" "-DBIN=${CMAKE_CURRENT_BINARY_DIR}/${chip}/${target}.bin"
            "-DOUT=${CMAKE_CURRENT_BINARY_DIR}/${chip}/manifest.json" "-DCHIP=${device}"
            "-DLIMIT=${flash_limit}" "-DVERSION=${firmware_version}"
            -P "${CMAKE_CURRENT_SOURCE_DIR}/cmake/write_manifest.cmake"
        WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
        VERBATIM)
endforeach()
