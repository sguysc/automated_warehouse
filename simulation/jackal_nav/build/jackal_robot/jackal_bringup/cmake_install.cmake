# Install script for directory: /home/cornell/Documents/Projects/AutomatedWarehouse/simulation/jackal_nav/src/jackal_robot/jackal_bringup

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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/cornell/Documents/Projects/AutomatedWarehouse/simulation/jackal_nav/build/jackal_robot/jackal_bringup/catkin_generated/installspace/jackal_bringup.pc")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/jackal_bringup/cmake" TYPE FILE FILES
    "/home/cornell/Documents/Projects/AutomatedWarehouse/simulation/jackal_nav/build/jackal_robot/jackal_bringup/catkin_generated/installspace/jackal_bringupConfig.cmake"
    "/home/cornell/Documents/Projects/AutomatedWarehouse/simulation/jackal_nav/build/jackal_robot/jackal_bringup/catkin_generated/installspace/jackal_bringupConfig-version.cmake"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/jackal_bringup" TYPE FILE FILES "/home/cornell/Documents/Projects/AutomatedWarehouse/simulation/jackal_nav/src/jackal_robot/jackal_bringup/package.xml")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  include("/home/cornell/Documents/Projects/AutomatedWarehouse/simulation/jackal_nav/build/jackal_robot/jackal_bringup/catkin_generated/safe_execute_install.cmake")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/jackal_bringup" TYPE DIRECTORY FILES "/home/cornell/Documents/Projects/AutomatedWarehouse/simulation/jackal_nav/src/jackal_robot/jackal_bringup/launch")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/jackal_bringup" TYPE PROGRAM FILES
    "/home/cornell/Documents/Projects/AutomatedWarehouse/simulation/jackal_nav/src/jackal_robot/jackal_bringup/scripts/install"
    "/home/cornell/Documents/Projects/AutomatedWarehouse/simulation/jackal_nav/src/jackal_robot/jackal_bringup/scripts/navsat_rtk_relay"
    )
endif()

