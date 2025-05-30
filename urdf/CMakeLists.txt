# Copyright (C) 2006-2023 Istituto Italiano di Tecnologia (IIT)
# All rights reserved.
#
# This software may be modified and distributed under the terms of the
# BSD-3-Clause license. See the accompanying LICENSE file for details.


# Detect if we are doing a standalone build of the r1-models
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
  cmake_minimum_required(VERSION 3.22.1)
  project(r1-models LANGUAGES NONE)
  set(BUILD_PREFIX "R1Mk3")
  include(GNUInstallDirs)
endif()

set(GAZEBO_SUPPORTED_MODELS "")
list(APPEND GAZEBO_SUPPORTED_MODELS "R1Mk3Gazebo")
list(APPEND GAZEBO_SUPPORTED_MODELS "R1Mk3Gazebo_minContacts")

set(INSTALLED_URDF_MODELS "")

macro(SUBDIRLIST result curdir)
  file(GLOB children RELATIVE ${curdir} ${curdir}/*)
  set(dirlist "")
  foreach(child ${children})
    if(IS_DIRECTORY ${curdir}/${child})
      list(APPEND dirlist ${child})
    endif()
  endforeach()
  set(${result} ${dirlist})
endmacro()

# Copy the R1Mk3 folder in the build tree
file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/${BUILD_PREFIX} DESTINATION ${CMAKE_BINARY_DIR})

set(R1MK3_MODELS_SDF_VERSION "1.7")

SUBDIRLIST(ROBOTS_NAMES ${CMAKE_CURRENT_SOURCE_DIR}/${BUILD_PREFIX}/robots)
foreach(ROBOT_DIRNAME ${ROBOTS_NAMES})
  set(ROBOT_NAME ${ROBOT_DIRNAME})
  set(ROBOT_MODEL_CONFIG_FILE "${CMAKE_BINARY_DIR}/${BUILD_PREFIX}/robots/${ROBOT_NAME}/model.config")
  set(ROBOT_FIXED_MODEL_FOLDER "${CMAKE_BINARY_DIR}/${BUILD_PREFIX}/robots/${ROBOT_NAME}_fixed")
  set(ROBOT_FIXED_MODEL_CONFIG_FILE "${ROBOT_FIXED_MODEL_FOLDER}/model.config")
  set(ROBOT_FIXED_MODEL_SDF_FILE "${ROBOT_FIXED_MODEL_FOLDER}/${ROBOT_NAME}_fixed.sdf")

  list(APPEND INSTALLED_URDF_MODELS \"${ROBOT_NAME}\")
  # When copying in sources remove the _fixed models
  if(ROBOT_NAME IN_LIST GAZEBO_SUPPORTED_MODELS)

    set(ROBOT_NAME_CONFIG "${ROBOT_NAME}")
    set(ROBOT_MODEL_CONFIG "model.urdf")
    set(ROBOT_MODEL_DESCRIPTION_CONFIG "Model for the R1Mk3 humanoid robot.")
    configure_file(${CMAKE_CURRENT_SOURCE_DIR}/model.config.in
                   ${ROBOT_MODEL_CONFIG_FILE}
                   @ONLY)

    file(MAKE_DIRECTORY ${ROBOT_FIXED_MODEL_FOLDER})
    set(ROBOT_NAME_CONFIG "${ROBOT_NAME}_fixed")
    set(ROBOT_MODEL_CONFIG "${ROBOT_NAME}_fixed.sdf")
    set(ROBOT_MODEL_DESCRIPTION_CONFIG "Model for the R1Mk3 humanoid robot, fixed on the air")
    configure_file(${CMAKE_CURRENT_SOURCE_DIR}/model.config.in
                   ${ROBOT_FIXED_MODEL_CONFIG_FILE}
                   @ONLY)
    configure_file(${CMAKE_CURRENT_SOURCE_DIR}/fixed_model.sdf.in
                   ${ROBOT_FIXED_MODEL_SDF_FILE}
                   @ONLY)
  endif()
endforeach()

# install the directory
install(DIRECTORY ${CMAKE_BINARY_DIR}/${BUILD_PREFIX} DESTINATION share)

# Install empty R1Mk3 file in <prefix>/share/ament_index/resource_index/packages
# so that R1Mk3 package is found by ROS2
# See https://github.com/robotology/icub-models/issues/177
# and https://github.com/icub-tech-iit/ergocub-software/issues/59
file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/R1Mk3_empty_file "")
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/R1Mk3_empty_file
        DESTINATION share/ament_index/resource_index/packages
        RENAME R1Mk3)