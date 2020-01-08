# Install script for directory: /home/cornell/Documents/Projects/AutomatedWarehouse/simulation/jackal_nav/src/multi_jackal_description

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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/cornell/Documents/Projects/AutomatedWarehouse/simulation/jackal_nav/build/multi_jackal_description/catkin_generated/installspace/multi_jackal_description.pc")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/multi_jackal_description/cmake" TYPE FILE FILES
    "/home/cornell/Documents/Projects/AutomatedWarehouse/simulation/jackal_nav/build/multi_jackal_description/catkin_generated/installspace/multi_jackal_descriptionConfig.cmake"
    "/home/cornell/Documents/Projects/AutomatedWarehouse/simulation/jackal_nav/build/multi_jackal_description/catkin_generated/installspace/multi_jackal_descriptionConfig-version.cmake"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/multi_jackal_description" TYPE FILE FILES "/home/cornell/Documents/Projects/AutomatedWarehouse/simulation/jackal_nav/src/multi_jackal_description/package.xml")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/multi_jackal_description" TYPE PROGRAM FILES "/home/cornell/Documents/Projects/AutomatedWarehouse/simulation/jackal_nav/src/multi_jackal_description/scripts/env_run")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/multi_jackal_description" TYPE DIRECTORY FILES
    "/home/cornell/Documents/Projects/AutomatedWarehouse/simulation/jackal_nav/src/multi_jackal_description/meshes"
    "/home/cornell/Documents/Projects/AutomatedWarehouse/simulation/jackal_nav/src/multi_jackal_description/launch"
    "/home/cornell/Documents/Projects/AutomatedWarehouse/simulation/jackal_nav/src/multi_jackal_description/urdf"
    )
endif()

