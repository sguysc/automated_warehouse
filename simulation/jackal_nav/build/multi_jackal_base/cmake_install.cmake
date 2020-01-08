# Install script for directory: /home/cornell/Documents/Projects/AutomatedWarehouse/simulation/jackal_nav/src/multi_jackal_base

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/cornell/Documents/Projects/AutomatedWarehouse/simulation/jackal_nav/install")
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

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/cornell/Documents/Projects/AutomatedWarehouse/simulation/jackal_nav/build/multi_jackal_base/catkin_generated/installspace/multi_jackal_base.pc")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/multi_jackal_base/cmake" TYPE FILE FILES
    "/home/cornell/Documents/Projects/AutomatedWarehouse/simulation/jackal_nav/build/multi_jackal_base/catkin_generated/installspace/multi_jackal_baseConfig.cmake"
    "/home/cornell/Documents/Projects/AutomatedWarehouse/simulation/jackal_nav/build/multi_jackal_base/catkin_generated/installspace/multi_jackal_baseConfig-version.cmake"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/multi_jackal_base" TYPE FILE FILES "/home/cornell/Documents/Projects/AutomatedWarehouse/simulation/jackal_nav/src/multi_jackal_base/package.xml")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/multi_jackal_base" TYPE DIRECTORY FILES "/home/cornell/Documents/Projects/AutomatedWarehouse/simulation/jackal_nav/src/multi_jackal_base/launch")
endif()

