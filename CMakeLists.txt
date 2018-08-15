cmake_minimum_required(VERSION 3.0)

# check c++11 / c++0x
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
   set(CMAKE_CXX_FLAGS "-std=c++11")
   message(COMPILER_SUPPORTS_CXX11)
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "-std=c++0x")
    message(COMPILER_SUPPORTS_CXX0X)
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

# Pass options to GCC
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fpic")

# This is a CMake custom_main for Python


FIND_PACKAGE(SWIG REQUIRED)
INCLUDE(${SWIG_USE_FILE})


FIND_PACKAGE(PythonLibs 2.7 REQUIRED)


FIND_PACKAGE(SWIG REQUIRED)
INCLUDE(${SWIG_USE_FILE})


# ROS import
find_package(roscpp REQUIRED)



# Depend on system install of Gazebo
find_package(gazebo REQUIRED)


# Numpy
if(NOT PYTHON_EXECUTABLE)
  if(NumPy_FIND_QUIETLY)
    find_package(PythonInterp QUIET)
  else()
    find_package(PythonInterp)
    set(__numpy_out 1)
  endif()
endif()
if (PYTHON_EXECUTABLE)
  # Find out the include path
  execute_process(
    COMMAND "${PYTHON_EXECUTABLE}" -c
            "from __future__ import print_function\ntry: import numpy; print(numpy.get_include(), end='')\nexcept:pass\n"
            OUTPUT_VARIABLE __numpy_path)
  # And the version
  execute_process(
    COMMAND "${PYTHON_EXECUTABLE}" -c
            "from __future__ import print_function\ntry: import numpy; print(numpy.__version__, end='')\nexcept:pass\n"
    OUTPUT_VARIABLE __numpy_version)
elseif(__numpy_out)
  message(STATUS "Python executable not found.")
endif(PYTHON_EXECUTABLE)
find_path(PYTHON_NUMPY_INCLUDE_DIR numpy/arrayobject.h
HINTS "${__numpy_path}" "${PYTHON_INCLUDE_PATH}" NO_DEFAULT_PATH)
if(PYTHON_NUMPY_INCLUDE_DIR)
  set(PYTHON_NUMPY_FOUND 1 CACHE INTERNAL "Python numpy found")
endif(PYTHON_NUMPY_INCLUDE_DIR)



INCLUDE_DIRECTORIES(${PYTHON_INCLUDE_PATH})
INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR})
INCLUDE_DIRECTORIES(${GAZEBO_INCLUDE_DIRS})
include_directories(include)
include_directories(${roscpp_INCLUDE_DIRS})
include_directories(${PYTHON_NUMPY_INCLUDE_DIR})



link_directories(${GAZEBO_LIBRARY_DIRS})


message("#############################")
message(${CMAKE_CURRENT_BINARY_DIR})

message("#############################")

SET(CMAKE_SWIG_FLAGS "")

SET_SOURCE_FILES_PROPERTIES(src/custom_main.i PROPERTIES CPLUSPLUS ON)
SET_SOURCE_FILES_PROPERTIES(src/custom_main.i PROPERTIES SWIG_FLAGS "")
SWIG_ADD_MODULE(custom_main python src/custom_main.i src/custom_main.cpp )
SWIG_LINK_LIBRARIES(custom_main ${PYTHON_LIBRARIES} ${GAZEBO_LIBRARIES} ${roscpp_LIBRARIES})


add_custom_command(TARGET _custom_main POST_BUILD
                   COMMAND touch ${CMAKE_CURRENT_BINARY_DIR}/__init__.py
)
