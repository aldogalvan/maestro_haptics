cmake_minimum_required(VERSION 3.20)
project(maestro_haptics)

set(CMAKE_CXX_STANDARD 14)

if(WIN32)
    set(OS "win32")
    include_directories(include/soem_include/oshw/win32/wpcap/Include)
    link_directories(${CMAKE_SOURCE_DIR}include/soem_include/oshw/win32/wpcap/Lib)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} /D _CRT_SECURE_NO_WARNINGS")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  /WX")
    set(OS_LIBS wpcap.lib Packet.lib Ws2_32.lib Winmm.lib)
elseif(UNIX)
    set(OS "linux")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra -Werror")
    set(OS_LIBS pthread rt)
elseif(${CMAKE_SYSTEM_NAME} MATCHES "rt-kernel")
    set(OS "rtk")
    message("ARCH is ${ARCH}")
    message("BSP is ${BSP}")
    include_directories(include/soem_include/oshw/${OS}/${ARCH})
    file(GLOB OSHW_EXTRA_SOURCES include/soem_include/oshw/${OS}/${ARCH}/*.c)
    set(OSHW_SOURCES "${OS_HW_SOURCES} ${OSHW_ARCHSOURCES}")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra -Werror")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wno-unused-but-set-variable")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wno-unused-function")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wno-format")
    set(OS_LIBS "-Wl,--start-group -l${BSP} -l${ARCH} -lkern -ldev -lsio -lblock -lfs -lusb -llwip -leth -li2c -lrtc -lcan -lnand -lspi -lnor -lpwm -ladc -ltrace -lc -lm -Wl,--end-group")
endif()

message("OS is ${OS}")

if (CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
    # Default to installing in SOEM source directory
    set(CMAKE_INSTALL_PREFIX ${CMAKE_SOURCE_DIR}/install)
endif()

file(GLOB SOEM_SOURCES include/soem_include/soem/*.c)
file(GLOB OSAL_SOURCES include/soem_include/osal/${OS}/*.c)
file(GLOB OSHW_SOURCES include/soem_include/oshw/${OS}/*.c)
file(GLOB ESMACAT_SOURCES include/esmacat_server/*.cpp)
file(GLOB MAESTRO3_SOURCES include/maestro3/*.cpp)

file(GLOB SOEM_HEADERS include/soem_include/soem/*.h)
file(GLOB OSAL_HEADERS include/soem_include/osal/osal.h osal/${OS}/*.h)
file(GLOB OSHW_HEADERS include/soem_include/oshw/${OS}/*.h)
file(GLOB ESMACAT_HEADERS esmacat_server/*.h)
file(GLOB MAESTRO3_HEADERS include/maestro3/*.h)

include_directories(include/soem_include/soem)
include_directories(include/soem_include/osal)
include_directories(include/soem_include/osal/${OS})
include_directories(include/soem_include/oshw/${OS})
include_directories(include/esmacat_server)
include_directories(include/maestro3)

#Generate the shared library from the sources
add_library(soem STATIC
        ${SOEM_SOURCES}
        ${OSAL_SOURCES}
        ${OSHW_SOURCES}
        ${OSHW_EXTRA_SOURCES})
target_link_libraries(soem ${OS_LIBS})

add_library(esmacat STATIC
        ${ESMACAT_SOURCES}
        )
#target_link_libraries(esmacat soem)

add_library(maestro3 STATIC
        ${MAESTRO3_SOURCES}
        )
target_link_libraries(maestro3 esmacat soem)

#include necessary libraries
set(OpenGL_GL_PREFERENCE GLVND)
find_package(OpenGL REQUIRED)
find_package(glfw3 3.3 REQUIRED)

#EIGEN3
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
list(PREPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/../cmake)

#LibIGL
list(PREPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
option(LIBIGL_WITH_OPENGL   "Use OpenGL"    ON)
option(LIBIGL_WITH_OPENGL_GLFW   "Use GLFW"    ON)
include(libigl)

#
# static library
#

# header search path
include_directories (src)

# static library source files
file (GLOB MAESTRO_HAPTICS_SOURCES src/*.cpp src/*.h)

# static library
add_library (maestro-haptics STATIC ${MAESTRO_HAPTICS_SOURCES})

# Link to libraries
target_link_libraries(maestro-haptics Eigen3::Eigen glfw OpenGL)
target_link_libraries(maestro-haptics igl::core igl::opengl_glfw)
target_link_libraries(maestro-haptics maestro3)

#install(FILES
#  ${SOEM_HEADERS}
#  ${OSAL_HEADERS}
#  ${OSHW_HEADERS}
#  DESTINATION ${SOEM_INCLUDE_INSTALL_DIR})

add_subdirectory(examples)

