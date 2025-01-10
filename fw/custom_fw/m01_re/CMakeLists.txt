#THIS FILE IS AUTO GENERATED FROM THE TEMPLATE! DO NOT CHANGE!
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_VERSION 1)
cmake_minimum_required(VERSION 3.30)

# specify cross-compilers and tools
# done using the CLion Toolchain settings
#set(CMAKE_C_COMPILER arm-none-eabi-gcc)
#set(CMAKE_CXX_COMPILER arm-none-eabi-g++)
#set(CMAKE_ASM_COMPILER  arm-none-eabi-gcc)
#set(CMAKE_AR arm-none-eabi-ar)
#set(CMAKE_OBJCOPY arm-none-eabi-objcopy)
#set(CMAKE_OBJDUMP arm-none-eabi-objdump)
#set(SIZE arm-none-eabi-size)

# cross compile for uC, so test with static lib
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

# project settings, c++ disabled for now
unset(CMAKE_CXX_COMPILER)
project(m01_re C ASM)
#set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_STANDARD 99)
#set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_C_EXTENSIONS OFF) # TODO may be needed, __attribute__ is used in HAL

##################################################################################################
######################################### common options #########################################

add_compile_options(
        -mcpu=cortex-m3 # STM32F103VCT6 is based on the cortex M3
        -mthumb # cortex M3 only supports thumb ISA
        -mfloat-abi=soft # STM32F103VCT6 doesn't have an FPU
        -flto # link time optimization


        -ffunction-sections # separate ELF section for functions
        -fdata-sections # separate ELF section for vars


        # use lightweight version of c std lib to reduce flash usage
        --specs=nano.specs


        # creates .cyclo files in the build dir. also help ensure the correct ST Micro toolchain
        # is used, as most toolchains (including arm's) don't usually include it
        -fcyclomatic-complexity

        -fno-common # avoid same global var being defined in multiple scr files

        #-mslow-flash-data # cache frequently accessed vars in ram

        # lots of warnings
        -Wall
        -Wextra
        -pedantic
        -Wswitch-default
        -Wswitch-enum
        -Wconversion
        -Wshadow
        -Wdouble-promotion

        -fstack-usage # generates .su files in build dir listing function stack usage
)


set(LINKER_SCRIPT ${CMAKE_SOURCE_DIR}/STM32F103VCTX_FLASH.ld)

add_link_options(
        -mcpu=cortex-m3 # STM32F103VCT6 is based on the cortex M3
        -mthumb # cortex M3 only supports thumb ISA
        -mfloat-abi=soft # STM32F103VCT6 doesn't have an FPU
        -Wl,-flto # link time optimization


        -Wl,--gc-sections # remove unused sections
        #-Wl,--print-gc-sections # print what is being thrown away


        # use lightweight version of c std lib to reduce flash usage
        --specs=nano.specs
        #-Wl,-u,_printf_float # "--specs=nano.specs" disables float fmt for printf/scanf. reenable with this
        #-Wl,-u,_scanf_float # "--specs=nano.specs" disables float fmt for printf/scanf. reenable with this


        # link c std, math, gcc support libs
        -Wl,--start-group
        -Wl,-lc
        -Wl,-lm
        -Wl,-lgcc
        -Wl,--end-group

        -Wl,-static # default anyways but still

        -Wl,--print-memory-usage # print usages after build

        --specs=nosys.specs # no underlying OS

        -Wl,-T ${LINKER_SCRIPT}

        -Wl,-Map=${PROJECT_BINARY_DIR}/${PROJECT_NAME}.map

)

add_definitions(-DUSE_HAL_DRIVER -DSTM32F103xE) # TODO maybe needs add_definitions(-DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xE) for new defines

######################################### common options #########################################
##################################################################################################

# Enable assembler files preprocessing
add_compile_options($<$<COMPILE_LANGUAGE:ASM>:-x$<SEMICOLON>assembler-with-cpp>)

if ("${CMAKE_BUILD_TYPE}" STREQUAL "Release")
    add_compile_options(-Ofast)
elseif ("${CMAKE_BUILD_TYPE}" STREQUAL "ReleaseDebug")
    add_definitions(-DDEBUG)
    add_compile_options(-Ofast -g3)
elseif ("${CMAKE_BUILD_TYPE}" STREQUAL "Size")
    add_compile_options(-Os)
elseif ("${CMAKE_BUILD_TYPE}" STREQUAL "MinSize")
    add_compile_options(-Oz)
elseif ("${CMAKE_BUILD_TYPE}" STREQUAL "Debug")
    add_definitions(-DDEBUG)
    add_compile_options(-Og -g3)
endif ()

include_directories(Core/Inc Drivers/STM32F1xx_HAL_Driver/Inc/Legacy Drivers/STM32F1xx_HAL_Driver/Inc Drivers/CMSIS/Device/ST/STM32F1xx/Include Drivers/CMSIS/Include)

file(GLOB_RECURSE SOURCES "Core/*.*" "Drivers/*.*")

add_executable(${PROJECT_NAME}.elf ${SOURCES} ${LINKER_SCRIPT})

set(HEX_FILE ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.hex)
set(BIN_FILE ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.bin)

add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
        COMMAND ${CMAKE_OBJCOPY} -Oihex $<TARGET_FILE:${PROJECT_NAME}.elf> ${HEX_FILE}
        COMMAND ${CMAKE_OBJCOPY} -Obinary $<TARGET_FILE:${PROJECT_NAME}.elf> ${BIN_FILE}
        COMMENT "Building ${HEX_FILE}
Building ${BIN_FILE}")
