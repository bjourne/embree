## ======================================================================== ##
## Copyright 2009-2018 Intel Corporation                                    ##
##                                                                          ##
## Licensed under the Apache License, Version 2.0 (the "License");          ##
## you may not use this file except in compliance with the License.         ##
## You may obtain a copy of the License at                                  ##
##                                                                          ##
##     http://www.apache.org/licenses/LICENSE-2.0                           ##
##                                                                          ##
## Unless required by applicable law or agreed to in writing, software      ##
## distributed under the License is distributed on an "AS IS" BASIS,        ##
## WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. ##
## See the License for the specific language governing permissions and      ##
## limitations under the License.                                           ##
## ======================================================================== ##

ADD_LIBRARY(tutorial STATIC tutorial.cpp application.cpp scene.cpp tutorial_device.cpp scene_device.cpp)
TARGET_LINK_LIBRARIES(tutorial sys math lexers scenegraph lights embree imgui ${OPENGL_LIBRARIES} ${GLFW_STATIC_LIBRARIES})
SET_PROPERTY(TARGET tutorial PROPERTY FOLDER tutorials/common)
SET_PROPERTY(TARGET tutorial APPEND PROPERTY COMPILE_FLAGS " ${FLAGS_LOWEST}")

IF (EMBREE_ISPC_SUPPORT)
  ADD_ISPC_LIBRARY(STATIC tutorial.cpp application.cpp scene.cpp tutorial_device.ispc scene_device.cpp tasksys.cpp)
  TARGET_LINK_LIBRARIES(sys math lexers scenegraph embree imgui ${OPENGL_LIBRARIES} ${GLFW_STATIC_LIBRARIES})
  SET_PROPERTY(TARGET tutorial_ispc PROPERTY FOLDER tutorials/common)
  SET_PROPERTY(TARGET tutorial_ispc APPEND PROPERTY COMPILE_FLAGS " ${FLAGS_LOWEST}")

ENDIF()

IF (WIN32)
  GET_FILENAME_COMPONENT(GLFW_DIR ${GLFW_LIBRARY} PATH)
  ADD_CUSTOM_COMMAND(TARGET tutorial POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different ${GLFW_DIR}/glfw3.dll $<TARGET_FILE_DIR:tutorial>
    COMMENT "Copying GLFW DLL" VERBATIM
  )
  INSTALL(PROGRAMS ${GLFW_DIR}/glfw3.dll DESTINATION ${CMAKE_INSTALL_BINDIR} COMPONENT examples)
ENDIF (WIN32)
