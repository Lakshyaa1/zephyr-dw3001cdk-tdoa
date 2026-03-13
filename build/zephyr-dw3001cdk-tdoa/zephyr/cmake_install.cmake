# Install script for directory: C:/ncs/v3.2.2/zephyr

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "C:/Program Files (x86)/Zephyr-Kernel")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "TRUE")
endif()

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "C:/ncs/toolchains/c717907b94/opt/zephyr-sdk/arm-zephyr-eabi/bin/arm-zephyr-eabi-objdump.exe")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/ncs/SDK_ASS/Examples/ncs-fund/zephyr-dw3001cdk-tdoa/build/zephyr-dw3001cdk-tdoa/zephyr/arch/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/ncs/SDK_ASS/Examples/ncs-fund/zephyr-dw3001cdk-tdoa/build/zephyr-dw3001cdk-tdoa/zephyr/lib/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/ncs/SDK_ASS/Examples/ncs-fund/zephyr-dw3001cdk-tdoa/build/zephyr-dw3001cdk-tdoa/zephyr/soc/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/ncs/SDK_ASS/Examples/ncs-fund/zephyr-dw3001cdk-tdoa/build/zephyr-dw3001cdk-tdoa/zephyr/boards/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/ncs/SDK_ASS/Examples/ncs-fund/zephyr-dw3001cdk-tdoa/build/zephyr-dw3001cdk-tdoa/zephyr/subsys/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/ncs/SDK_ASS/Examples/ncs-fund/zephyr-dw3001cdk-tdoa/build/zephyr-dw3001cdk-tdoa/zephyr/drivers/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/ncs/SDK_ASS/Examples/ncs-fund/zephyr-dw3001cdk-tdoa/build/zephyr-dw3001cdk-tdoa/modules/nrf/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/ncs/SDK_ASS/Examples/ncs-fund/zephyr-dw3001cdk-tdoa/build/zephyr-dw3001cdk-tdoa/modules/hostap/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/ncs/SDK_ASS/Examples/ncs-fund/zephyr-dw3001cdk-tdoa/build/zephyr-dw3001cdk-tdoa/modules/mcuboot/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/ncs/SDK_ASS/Examples/ncs-fund/zephyr-dw3001cdk-tdoa/build/zephyr-dw3001cdk-tdoa/modules/mbedtls/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/ncs/SDK_ASS/Examples/ncs-fund/zephyr-dw3001cdk-tdoa/build/zephyr-dw3001cdk-tdoa/modules/trusted-firmware-m/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/ncs/SDK_ASS/Examples/ncs-fund/zephyr-dw3001cdk-tdoa/build/zephyr-dw3001cdk-tdoa/modules/cjson/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/ncs/SDK_ASS/Examples/ncs-fund/zephyr-dw3001cdk-tdoa/build/zephyr-dw3001cdk-tdoa/modules/azure-sdk-for-c/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/ncs/SDK_ASS/Examples/ncs-fund/zephyr-dw3001cdk-tdoa/build/zephyr-dw3001cdk-tdoa/modules/cirrus-logic/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/ncs/SDK_ASS/Examples/ncs-fund/zephyr-dw3001cdk-tdoa/build/zephyr-dw3001cdk-tdoa/modules/openthread/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/ncs/SDK_ASS/Examples/ncs-fund/zephyr-dw3001cdk-tdoa/build/zephyr-dw3001cdk-tdoa/modules/memfault-firmware-sdk/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/ncs/SDK_ASS/Examples/ncs-fund/zephyr-dw3001cdk-tdoa/build/zephyr-dw3001cdk-tdoa/modules/canopennode/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/ncs/SDK_ASS/Examples/ncs-fund/zephyr-dw3001cdk-tdoa/build/zephyr-dw3001cdk-tdoa/modules/chre/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/ncs/SDK_ASS/Examples/ncs-fund/zephyr-dw3001cdk-tdoa/build/zephyr-dw3001cdk-tdoa/modules/lz4/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/ncs/SDK_ASS/Examples/ncs-fund/zephyr-dw3001cdk-tdoa/build/zephyr-dw3001cdk-tdoa/modules/zscilib/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/ncs/SDK_ASS/Examples/ncs-fund/zephyr-dw3001cdk-tdoa/build/zephyr-dw3001cdk-tdoa/modules/cmsis/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/ncs/SDK_ASS/Examples/ncs-fund/zephyr-dw3001cdk-tdoa/build/zephyr-dw3001cdk-tdoa/modules/cmsis-dsp/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/ncs/SDK_ASS/Examples/ncs-fund/zephyr-dw3001cdk-tdoa/build/zephyr-dw3001cdk-tdoa/modules/cmsis-nn/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/ncs/SDK_ASS/Examples/ncs-fund/zephyr-dw3001cdk-tdoa/build/zephyr-dw3001cdk-tdoa/modules/cmsis_6/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/ncs/SDK_ASS/Examples/ncs-fund/zephyr-dw3001cdk-tdoa/build/zephyr-dw3001cdk-tdoa/modules/fatfs/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/ncs/SDK_ASS/Examples/ncs-fund/zephyr-dw3001cdk-tdoa/build/zephyr-dw3001cdk-tdoa/modules/hal_nordic/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/ncs/SDK_ASS/Examples/ncs-fund/zephyr-dw3001cdk-tdoa/build/zephyr-dw3001cdk-tdoa/modules/hal_st/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/ncs/SDK_ASS/Examples/ncs-fund/zephyr-dw3001cdk-tdoa/build/zephyr-dw3001cdk-tdoa/modules/hal_tdk/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/ncs/SDK_ASS/Examples/ncs-fund/zephyr-dw3001cdk-tdoa/build/zephyr-dw3001cdk-tdoa/modules/hal_wurthelektronik/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/ncs/SDK_ASS/Examples/ncs-fund/zephyr-dw3001cdk-tdoa/build/zephyr-dw3001cdk-tdoa/modules/liblc3/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/ncs/SDK_ASS/Examples/ncs-fund/zephyr-dw3001cdk-tdoa/build/zephyr-dw3001cdk-tdoa/modules/libmetal/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/ncs/SDK_ASS/Examples/ncs-fund/zephyr-dw3001cdk-tdoa/build/zephyr-dw3001cdk-tdoa/modules/littlefs/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/ncs/SDK_ASS/Examples/ncs-fund/zephyr-dw3001cdk-tdoa/build/zephyr-dw3001cdk-tdoa/modules/loramac-node/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/ncs/SDK_ASS/Examples/ncs-fund/zephyr-dw3001cdk-tdoa/build/zephyr-dw3001cdk-tdoa/modules/lvgl/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/ncs/SDK_ASS/Examples/ncs-fund/zephyr-dw3001cdk-tdoa/build/zephyr-dw3001cdk-tdoa/modules/mipi-sys-t/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/ncs/SDK_ASS/Examples/ncs-fund/zephyr-dw3001cdk-tdoa/build/zephyr-dw3001cdk-tdoa/modules/nanopb/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/ncs/SDK_ASS/Examples/ncs-fund/zephyr-dw3001cdk-tdoa/build/zephyr-dw3001cdk-tdoa/modules/nrf_wifi/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/ncs/SDK_ASS/Examples/ncs-fund/zephyr-dw3001cdk-tdoa/build/zephyr-dw3001cdk-tdoa/modules/open-amp/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/ncs/SDK_ASS/Examples/ncs-fund/zephyr-dw3001cdk-tdoa/build/zephyr-dw3001cdk-tdoa/modules/percepio/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/ncs/SDK_ASS/Examples/ncs-fund/zephyr-dw3001cdk-tdoa/build/zephyr-dw3001cdk-tdoa/modules/picolibc/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/ncs/SDK_ASS/Examples/ncs-fund/zephyr-dw3001cdk-tdoa/build/zephyr-dw3001cdk-tdoa/modules/segger/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/ncs/SDK_ASS/Examples/ncs-fund/zephyr-dw3001cdk-tdoa/build/zephyr-dw3001cdk-tdoa/modules/uoscore-uedhoc/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/ncs/SDK_ASS/Examples/ncs-fund/zephyr-dw3001cdk-tdoa/build/zephyr-dw3001cdk-tdoa/modules/zcbor/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/ncs/SDK_ASS/Examples/ncs-fund/zephyr-dw3001cdk-tdoa/build/zephyr-dw3001cdk-tdoa/modules/nrfxlib/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/ncs/SDK_ASS/Examples/ncs-fund/zephyr-dw3001cdk-tdoa/build/zephyr-dw3001cdk-tdoa/modules/nrf_hw_models/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/ncs/SDK_ASS/Examples/ncs-fund/zephyr-dw3001cdk-tdoa/build/zephyr-dw3001cdk-tdoa/modules/connectedhomeip/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/ncs/SDK_ASS/Examples/ncs-fund/zephyr-dw3001cdk-tdoa/build/zephyr-dw3001cdk-tdoa/zephyr/kernel/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/ncs/SDK_ASS/Examples/ncs-fund/zephyr-dw3001cdk-tdoa/build/zephyr-dw3001cdk-tdoa/zephyr/cmake/flash/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/ncs/SDK_ASS/Examples/ncs-fund/zephyr-dw3001cdk-tdoa/build/zephyr-dw3001cdk-tdoa/zephyr/cmake/usage/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("C:/ncs/SDK_ASS/Examples/ncs-fund/zephyr-dw3001cdk-tdoa/build/zephyr-dw3001cdk-tdoa/zephyr/cmake/reports/cmake_install.cmake")
endif()

