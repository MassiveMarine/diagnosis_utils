#.rst:
# FindProtobuf
# ------------
#
# Locate and configure the Google Protocol Buffers library.
#
# The following variables can be set and are optional:
#
# ``PROTOBUF_SRC_ROOT_FOLDER``
#   When compiling with MSVC, if this cache variable is set
#   the protobuf-default VS project build locations
#   (vsprojects/Debug and vsprojects/Release
#   or vsprojects/x64/Debug and vsprojects/x64/Release)
#   will be searched for libraries and binaries.
# ``PROTOBUF_IMPORT_DIRS``
#   List of additional directories to be searched for
#   imported .proto files.
#
# Defines the following variables:
#
# ``PROTOBUF_FOUND``
#   Found the Google Protocol Buffers library
#   (libprotobuf & header files)
# ``PROTOBUF_INCLUDE_DIRS``
#   Include directories for Google Protocol Buffers
# ``PROTOBUF_LIBRARIES``
#   The protobuf libraries
# ``PROTOBUF_PROTOC_LIBRARIES``
#   The protoc libraries
# ``PROTOBUF_LITE_LIBRARIES``
#   The protobuf-lite libraries
#
# The following cache variables are also available to set or use:
#
# ``PROTOBUF_LIBRARY``
#   The protobuf library
# ``PROTOBUF_PROTOC_LIBRARY``
#   The protoc library
# ``PROTOBUF_INCLUDE_DIR``
#   The include directory for protocol buffers
# ``PROTOBUF_PROTOC_EXECUTABLE``
#   The protoc compiler
# ``PROTOBUF_LIBRARY_DEBUG``
#   The protobuf library (debug)
# ``PROTOBUF_PROTOC_LIBRARY_DEBUG``
#   The protoc library (debug)
# ``PROTOBUF_LITE_LIBRARY``
#   The protobuf lite library
# ``PROTOBUF_LITE_LIBRARY_DEBUG``
#   The protobuf lite library (debug)
#
# Example:
#
# .. code-block:: cmake
#
#   find_package(Protobuf REQUIRED)
#   include_directories(${PROTOBUF_INCLUDE_DIRS})
#   include_directories(${CMAKE_CURRENT_BINARY_DIR})
#   protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS foo.proto)
#   protobuf_generate_python(PROTO_PY foo.proto)
#   add_executable(bar bar.cc ${PROTO_SRCS} ${PROTO_HDRS})
#   target_link_libraries(bar ${PROTOBUF_LIBRARIES})
#
# .. note::
#   The ``protobuf_generate_cpp`` and ``protobuf_generate_python``
#   functions and :command:`add_executable` or :command:`add_library`
#   calls only work properly within the same directory.
#
# .. command:: protobuf_generate_cpp
#
#   Add custom commands to process ``.proto`` files to C++::
#
#     protobuf_generate_cpp (<SRCS> <HDRS> [<ARGN>...])
#
#   ``SRCS``
#     Variable to define with autogenerated source files
#   ``HDRS``
#     Variable to define with autogenerated header files
#   ``ARGN``
#     ``.proto`` files
#
# .. command:: protobuf_generate_python
#
#   Add custom commands to process ``.proto`` files to Python::
#
#     protobuf_generate_python (<PY> [<ARGN>...])
#
#   ``PY``
#     Variable to define with autogenerated Python files
#   ``ARGN``
#     ``.proto`` filess

#=============================================================================
# Copyright 2009 Kitware, Inc.
# Copyright 2009-2011 Philip Lowman <philip@yhbt.com>
# Copyright 2008 Esben Mose Hansen, Ange Optimization ApS
#
# Distributed under the OSI-approved BSD License (the "License");
# see accompanying file Copyright.txt for details.
#
# This software is distributed WITHOUT ANY WARRANTY; without even the
# implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
# See the License for more information.
#=============================================================================
# (To distribute this file outside of CMake, substitute the full
#  License text for the above reference.)

find_library(PROTO_C protoc /usr/local/lib)

if(PROTO_C)
    message("proto buffer lib installed")
    set(PROTO_BUF_FOUND true)
else()
    set(PROTO_BUF_FOUND false)
    message("Error: NO proto buffer lib installed")
endif()

# function(PROTOBUF_GENERATE SRCS HDRS PYS)
function(PROTOBUF_GENERATE)
  
  set(proto_dir msg)
  set(PROTO_FILES "")

  if(NOT ARGN)
    file(GLOB proto_files RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} "${proto_dir}/*.proto")
    list(APPEND PROTO_FILES ${proto_files})
  else()
    list(APPEND PROTO_FILES "${proto_dir}/${ARGN}")
  endif()  

  if(NOT PROTO_FILES)
    message(SEND_ERROR "Error: PROTOBUF_GENERATE() called without any proto files")
    return()
  endif()

  if(PROTOBUF_GENERATE_CPP_APPEND_PATH)
    # Create an include path for each file specified
    foreach(FIL ${PROTO_FILES})
      get_filename_component(ABS_FIL ${FIL} ABSOLUTE)
      get_filename_component(ABS_PATH ${ABS_FIL} PATH)
      list(FIND _protobuf_include_path ${ABS_PATH} _contains_already)
      if(${_contains_already} EQUAL -1)
          list(APPEND _protobuf_include_path -I ${ABS_PATH})
      endif()
    endforeach()
  else()
    set(_protobuf_include_path -I ${CMAKE_CURRENT_SOURCE_DIR})
  endif()

  if(DEFINED PROTOBUF_IMPORT_DIRS)
    foreach(DIR ${PROTOBUF_IMPORT_DIRS})
      get_filename_component(ABS_PATH ${DIR} ABSOLUTE)
      list(FIND _protobuf_include_path ${ABS_PATH} _contains_already)
      if(${_contains_already} EQUAL -1)
          list(APPEND _protobuf_include_path -I ${ABS_PATH})
      endif()
    endforeach()
  endif()

  set(SRCS "")
  set(HDRS "")
  set(PYS "")
  set(PROTOBUF_GENERATOR_HEADER_DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}/${PROJECT_NAME})
  set(PROTOBUF_GENERATOR_PYTHON_DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_PYTHON_DESTINATION}/${PROJECT_NAME})
  set(PROTOBUF_GENERATOR_PYTHON_DESTINATION_MSG ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_PYTHON_DESTINATION}/${PROJECT_NAME}/msg)
  file(MAKE_DIRECTORY ${PROTOBUF_GENERATOR_HEADER_DESTINATION} ${PROTOBUF_GENERATOR_PYTHON_DESTINATION} ${PROTOBUF_GENERATOR_PYTHON_DESTINATION_MSG})

  if(NOT EXISTS ${PROTOBUF_GENERATOR_PYTHON_DESTINATION}/__init__.py)
    file(WRITE ${PROTOBUF_GENERATOR_PYTHON_DESTINATION}/__init__.py "")
  endif()

  # if(NOT EXISTS ${PROTOBUF_GENERATOR_PYTHON_DESTINATION_MSG}/__init__.py)
  file(WRITE ${PROTOBUF_GENERATOR_PYTHON_DESTINATION_MSG}/__init__.py "")
  # endif()

  foreach(FIL ${PROTO_FILES})
    get_filename_component(ABS_FIL ${FIL} ABSOLUTE)
    get_filename_component(FIL_WE ${FIL} NAME_WE)

    list(APPEND SRCS "${PROTOBUF_GENERATOR_HEADER_DESTINATION}/${FIL_WE}.pb.cc")
    list(APPEND HDRS "${PROTOBUF_GENERATOR_HEADER_DESTINATION}/${FIL_WE}.pb.h")
    list(APPEND PYS "${PROTOBUF_GENERATOR_PYTHON_DESTINATION_MSG}/${FIL_WE}_pb2.py")

    file(APPEND ${PROTOBUF_GENERATOR_PYTHON_DESTINATION_MSG}/__init__.py "from .${FIL_WE}_pb2 import *")

    add_custom_command(
      OUTPUT "${PROTOBUF_GENERATOR_HEADER_DESTINATION}/${FIL_WE}.pb.cc"
             "${PROTOBUF_GENERATOR_HEADER_DESTINATION}/${FIL_WE}.pb.h"
             "${PROTOBUF_GENERATOR_PYTHON_DESTINATION_MSG}/${FIL_WE}_pb2.py"
      COMMAND  ${PROTOBUF_PROTOC_EXECUTABLE} ARGS --python_out ${PROTOBUF_GENERATOR_PYTHON_DESTINATION_MSG}
               --cpp_out  ${PROTOBUF_GENERATOR_HEADER_DESTINATION} ${_protobuf_include_path} ${ABS_FIL}
      DEPENDS ${ABS_FIL} ${PROTOBUF_PROTOC_EXECUTABLE}
      COMMENT "Running protocol buffer compiler (C++ and Python) on ${FIL}"
      VERBATIM )
  endforeach()

  set_source_files_properties(${SRCS} ${HDRS} PROPERTIES GENERATED TRUE)
  add_library(${PROJECT_NAME}_proto ${HDRS} ${SRCS})
  target_link_libraries(${PROJECT_NAME}_proto protobuf)
  target_compile_options(${PROJECT_NAME}_proto INTERFACE -I${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION})
endfunction()

if(CMAKE_SIZEOF_VOID_P EQUAL 8)
  set(_PROTOBUF_ARCH_DIR x64/)
endif()

# Internal function: search for normal library as well as a debug one
#    if the debug one is specified also include debug/optimized keywords
#    in *_LIBRARIES variable
function(_protobuf_find_libraries name filename)
   find_library(${name}_LIBRARY
       NAMES ${filename}
       PATHS ${PROTOBUF_SRC_ROOT_FOLDER}/vsprojects/${_PROTOBUF_ARCH_DIR}Release)
   mark_as_advanced(${name}_LIBRARY)

   find_library(${name}_LIBRARY_DEBUG
       NAMES ${filename}
       PATHS ${PROTOBUF_SRC_ROOT_FOLDER}/vsprojects/${_PROTOBUF_ARCH_DIR}Debug)
   mark_as_advanced(${name}_LIBRARY_DEBUG)

   if(NOT ${name}_LIBRARY_DEBUG)
      # There is no debug library
      set(${name}_LIBRARY_DEBUG ${${name}_LIBRARY} PARENT_SCOPE)
      set(${name}_LIBRARIES     ${${name}_LIBRARY} PARENT_SCOPE)
   else()
      # There IS a debug library
      set(${name}_LIBRARIES
          optimized ${${name}_LIBRARY}
          debug     ${${name}_LIBRARY_DEBUG}
          PARENT_SCOPE
      )
   endif()
endfunction()

# Internal function: find threads library
function(_protobuf_find_threads)
    set(CMAKE_THREAD_PREFER_PTHREAD TRUE)
    find_package(Threads)
    if(Threads_FOUND)
        list(APPEND PROTOBUF_LIBRARIES ${CMAKE_THREAD_LIBS_INIT})
        set(PROTOBUF_LIBRARIES "${PROTOBUF_LIBRARIES}" PARENT_SCOPE)
    endif()
endfunction()

#
# Main.
#

# By default have PROTOBUF_GENERATE_CPP macro pass -I to protoc
# for each directory where a proto file is referenced.
if(NOT DEFINED PROTOBUF_GENERATE_CPP_APPEND_PATH)
  set(PROTOBUF_GENERATE_CPP_APPEND_PATH TRUE)
endif()


# Google's provided vcproj files generate libraries with a "lib"
# prefix on Windows
if(MSVC)
    set(PROTOBUF_ORIG_FIND_LIBRARY_PREFIXES "${CMAKE_FIND_LIBRARY_PREFIXES}")
    set(CMAKE_FIND_LIBRARY_PREFIXES "lib" "")

    find_path(PROTOBUF_SRC_ROOT_FOLDER protobuf.pc.in)
endif()

# The Protobuf library
_protobuf_find_libraries(PROTOBUF protobuf)
#DOC "The Google Protocol Buffers RELEASE Library"

_protobuf_find_libraries(PROTOBUF_LITE protobuf-lite)

# The Protobuf Protoc Library
_protobuf_find_libraries(PROTOBUF_PROTOC protoc)

# Restore original find library prefixes
if(MSVC)
    set(CMAKE_FIND_LIBRARY_PREFIXES "${PROTOBUF_ORIG_FIND_LIBRARY_PREFIXES}")
endif()

if(UNIX)
    _protobuf_find_threads()
endif()

# Find the include directory
find_path(PROTOBUF_INCLUDE_DIR
    google/protobuf/service.h
    PATHS ${PROTOBUF_SRC_ROOT_FOLDER}/src
)
mark_as_advanced(PROTOBUF_INCLUDE_DIR)

# Find the protoc Executable
find_program(PROTOBUF_PROTOC_EXECUTABLE
    NAMES protoc
    DOC "The Google Protocol Buffers Compiler"
    PATHS
    ${PROTOBUF_SRC_ROOT_FOLDER}/vsprojects/${_PROTOBUF_ARCH_DIR}Release
    ${PROTOBUF_SRC_ROOT_FOLDER}/vsprojects/${_PROTOBUF_ARCH_DIR}Debug
)
mark_as_advanced(PROTOBUF_PROTOC_EXECUTABLE)


include(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(Protobuf DEFAULT_MSG
    PROTOBUF_LIBRARY PROTOBUF_INCLUDE_DIR)

if(PROTOBUF_FOUND)
    set(PROTOBUF_INCLUDE_DIRS ${PROTOBUF_INCLUDE_DIR})
endif()
