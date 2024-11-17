cmake_minimum_required(VERSION 3.0.2)
set(PROJ_NAME ProtocolMasterSlaveDev)
project(${PROJ_NAME})
set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED True)
add_definitions(-DFREERTOS_V10)

set(ROOT_PATH ${CMAKE_CURRENT_SOURCE_DIR})

message("Start building project...")

set(BACK_FROM_ROOT "../..")
set(FREERTOS_SRCS "${ROOT_PATH}/${BACK_FROM_ROOT}/Win32-simulator-MSVC/FreeRTOSv10_Source")
set(FREERTOS_EXPERIMENTS "${ROOT_PATH}/${BACK_FROM_ROOT}/")

include(${FREERTOS_EXPERIMENTS}/Configs.cmake)
message(STATUS ".cmake path: " ${MAIN_PROJECT_AND_CONFIGS_PATH})

set(FREERTOS_INC "${FREERTOS_SRCS}/include")
set(FREERTOS_PORTABLE "${FREERTOS_SRCS}/portable")
set(FREERTOS_SUPPORT_FUNCS "${ROOT_PATH}/${BACK_FROM_ROOT}/Win32-simulator-MSVC/Supporting_Functions")
set(EXTERNAL_LIBS "${ROOT_PATH}/${BACK_FROM_ROOT}/ExternalLibs")
set(BITLOGGER ${EXTERNAL_LIBS}/BitLogger)
set(SIMPLETIMER SpecLibs)
set(HARDWAREINTERFACE_UNIT ${ROOT_PATH}/SpecLibs/HardwareInterfaceUnit)
#set(CMSIS_OS_ENABLE_FLAG FALSE) #disable this flag if you don't want use cmsis on current project
if(CMSIS_OS_ENABLE_FLAG)
    set(CMSIS_OS ${ROOT_PATH}/cmsis_os_for_testing)
endif()


set(SOURCES
    ${FREERTOS_SRCS}/event_groups.c
    ${FREERTOS_SRCS}/list.c
    ${FREERTOS_SRCS}/queue.c
    ${FREERTOS_SRCS}/tasks.c
    ${FREERTOS_SRCS}/timers.c
    ${FREERTOS_SRCS}/croutine.c

    ${FREERTOS_PORTABLE}/MemMang/heap_4.c
    ${FREERTOS_PORTABLE}/MSVC-MingW/port.c
    ${FREERTOS_SUPPORT_FUNCS}/supporting_functions.c

    ${BITLOGGER}/BitLogger.c
    ${SIMPLETIMER}/SimpleTimerWP.c
    ${HARDWAREINTERFACE_UNIT}/HardwareInterfaceUnit.c
    ${ROOT_PATH}/main.c

    ${ROOT_PATH}/MicrochipPIC_Platform/portXC8.c
)

set(HEADERS
    ${FREERTOS_INC}/croutine.h
    ${FREERTOS_INC}/deprecated_definitions.h
    ${FREERTOS_INC}/event_groups.h
    ${FREERTOS_INC}/FreeRTOS.h
    ${FREERTOS_INC}/portable.h
    ${FREERTOS_INC}/list.h
    ${FREERTOS_INC}/mpu_wrappers.h
    ${FREERTOS_INC}/projdefs.h
    ${FREERTOS_INC}/queue.h
    ${FREERTOS_INC}/semphr.h
    #${FREERTOS_INC}/StackMacros.h #v8.2.3
    ${FREERTOS_INC}/stack_macros.h
    ${FREERTOS_INC}/task.h
    ${FREERTOS_INC}/timers.h

    ${FREERTOS_PORTABLE}/MSVC-MingW/portmacro.h
    ${FREERTOS_SUPPORT_FUNCS}/supporting_functions.h
    ${ROOT_PATH}/FreeRTOSConfig.h
    ${ROOT_PATH}/debug_print.h

    ${EXTERNAL_LIBS}/type_def.h
    ${BITLOGGER}/BitLogger.h
    ${SIMPLETIMER}/SimpleTimerWP.h
    ${HARDWAREINTERFACE_UNIT}/HardwareInterfaceUnit.h
)
add_definitions(-DFREERTOSCONFIG_RELATIVE_PATH="${ROOT_PATH}/FreeRTOSConfig.h")

add_executable(${PROJ_NAME} ${SOURCES} ${HEADERS})

if(CMSIS_OS_ENABLE_FLAG)
    set(CMSIS_OS_SRCHDR ${CMSIS_OS}/cmsis_os.c ${CMSIS_OS}/cmsis_os.h) 
    add_library(cmsis_os_library STATIC ${CMSIS_OS_SRCHDR})
    set_target_properties(cmsis_os_library PROPERTIES
         LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})
    target_link_libraries(${PROJ_NAME} cmsis_os_library)
endif()

if(${CMAKE_VERSION} VERSION_LESS "2.8.12")
    include_directories(${PROJ_NAME} PUBLIC ${INCLUDE_DIR})
else()
    target_include_directories(${PROJ_NAME} PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>/include)
endif()

target_link_options(${PROJ_NAME} PRIVATE "/MAP:${PROJECT_SOURCE_DIR}/outputProject.map")
