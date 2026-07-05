########################################################################
# Project setup
########################################################################
cmake_minimum_required(VERSION 3.25)
project(SoapyHPSDR CXX)
enable_testing()

#select the release build type by default to get optimization flags
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
    message(STATUS "Build type not specified: defaulting to release.")
endif(NOT CMAKE_BUILD_TYPE)
set(CMAKE_BUILD_TYPE ${CMAKE_BUILD_TYPE} CACHE STRING "")
if (CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    set(MINIMUM_GCC_VERSION "12.2")
    if (CMAKE_CXX_COMPILER_VERSION VERSION_LESS ${MINIMUM_GCC_VERSION})
        message(FATAL_ERROR "GCC version must be at least ${MINIMUM_GCC_VERSION}. You are using ${CMAKE_CXX_COMPILER_VERSION}.")
    endif()
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
########################################################################
# build the module
########################################################################
find_package(SoapySDR CONFIG)

if (NOT SoapySDR_FOUND)
    message(WARNING "SoapySDR development files not found - skipping support")
    return()
endif ()

SOAPY_SDR_MODULE_UTIL(
    TARGET SoapyHPSDR
    SOURCES SoapyHPSDR.cpp
    SoapyHPSDR.hpp
    SoapyHPSDRSettings.cpp
    SoapyHPSDRStreaming.cpp
    ReceiveThread.cpp
    ReceiveThread.hpp
    DataBuffer.h
)
#LIBRARIES ${MY_DEVICE_LIBRARIES}