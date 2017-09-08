cmake_minimum_required (VERSION 3.5.1)

set(CMAKE_VERBOSE_MAKEFILE ON)
set(CMAKE_COLOR_MAKEFILE   ON)

#SET PROJECT NAME
SET(PROJECT_NAME_ORIG "mavlink_cpp" )

#SET DEPENDENCIES DIRECTORY
SET(MODULES_DIR "modules" )

message("Building ${PROJECT_NAME_ORIG}...")

#BEGIN RESOLVE GIT REVISION
#https://bravenewmethod.com/2017/07/14/git-revision-as-compiler-definition-in-build-with-cmake/
message(STATUS "Resolving GIT Version...")
set(_build_version "unknown")
find_package(Git)
if(GIT_FOUND)
  execute_process(
    COMMAND ${GIT_EXECUTABLE} rev-parse --short HEAD
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
    OUTPUT_VARIABLE _build_version
    ERROR_QUIET
    OUTPUT_STRIP_TRAILING_WHITESPACE
  )
  message( STATUS "GIT hash: ${_build_version}")
else()
  message(STATUS "GIT not found")
endif()
string(TIMESTAMP _time_stamp)
#END RESOLVE GIT REVISION

#http://stackoverflow.com/questions/7787823/cmake-how-to-get-the-name-of-all-subdirectories-of-a-directory
function(GETPROJECTS result curdir)
  FILE(GLOB children RELATIVE ${curdir} ${curdir}/*/CMakeLists.txt)
  SET(dirlist "")
  FOREACH(file ${children})
        get_filename_component(subdir ${file} DIRECTORY)
        LIST(APPEND dirlist ${subdir})
  ENDFOREACH()
  SET(${result} ${dirlist} PARENT_SCOPE)
endfunction()

#DECLARE PROJECT
SET(PROJECT_NAME_GIT "${PROJECT_NAME_ORIG}_${_build_version}")
string(TOUPPER ${PROJECT_NAME_ORIG} TMP0)
SET(${TMP0}_GIT "${PROJECT_NAME_GIT}" PARENT_SCOPE)

if (TARGET ${PROJECT_NAME_GIT})
    message(WARNING "${PROJECT_NAME_GIT} has already been built!")
    return()
endif()

project (${PROJECT_NAME_GIT} CXX)
message(STATUS "PROJECT NAME: ${PROJECT_NAME}")

##################################
####### BUILD DEPENDENCIES #######
##################################

SET(DEPS_DIR ${MODULES_DIR})
SET(TMP0 ${CMAKE_CURRENT_SOURCE_DIR}/${DEPS_DIR})
GETPROJECTS(DEPS ${TMP0})
message(STATUS "Dependencies found in ${TMP0}: ${DEPS}")
FOREACH(subdir ${DEPS})
    add_subdirectory(${DEPS_DIR}/${subdir})
ENDFOREACH()

##################################
######### BUILD PROJECT ##########
##################################

SET(${PROJECT_NAME}_CXX_COMPILE_FLAGS "-fPIC -std=c++14")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${${PROJECT_NAME}_CXX_COMPILE_FLAGS}")

SET(SRCPATH src/)
SET(${PROJECT_NAME}_INCLUDE_DIRS
    ${SRCPATH}/lib/include
)

include_directories(
  ${${PROJECT_NAME}_INCLUDE_DIRS}
)

SET(${PROJECT_NAME}_CPP_FILES
    ${SRCPATH}/lib/GCS.cpp)

add_library(${PROJECT_NAME} SHARED ${${PROJECT_NAME}_CPP_FILES})
target_include_directories(${PROJECT_NAME} PUBLIC ${${PROJECT_NAME}_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} ${CPPLOGGING_GIT})

SET(MAINPATH ${SRCPATH}main)

add_executable(test0 ${MAINPATH}/test0.cpp)
target_link_libraries(test0 ${PROJECT_NAME} ${CPPLOGGING_GIT})

add_executable(testing ${MAINPATH}/mavlink_udp.cpp)
target_include_directories(testing PUBLIC ${${PROJECT_NAME}_INCLUDE_DIRS})
target_link_libraries(testing pthread)

message("End ${PROJECT_NAME_ORIG}")
