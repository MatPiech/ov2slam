cmake_minimum_required(VERSION 3.1 FATAL_ERROR)
project(ibow_lcd LANGUAGES CXX)

# Setting build type to Release
# if(NOT CMAKE_BUILD_TYPE)
#   set(CMAKE_BUILD_TYPE Release)
# endif()

message(STATUS "Build type: " ${CMAKE_BUILD_TYPE})

# Setting the flags for profiling information or not
if(CMAKE_BUILD_TYPE MATCHES Release)
  message(STATUS "Setting Release options")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -O3")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O3")
elseif(CMAKE_BUILD_TYPE MATCHES Debug)
  message(STATUS "Setting Debug options")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -O1 -pg")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O1 -pg")
endif()

# Check C++11 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)

set(CMAKE_CXX_STANDARD 14)

# if(COMPILER_SUPPORTS_CXX11)
# #    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
# #    message(STATUS "Using flag -std=c++11.")
# elseif(COMPILER_SUPPORTS_CXX0X)
# #    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
# #    message(STATUS "Using flag -std=c++0x.")
# else()
#    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support.
#                          Please use a different C++ compiler.")
# endif()

# Printing the compiling flags
message(STATUS "Compiler flags: ${CMAKE_CXX_FLAGS}")

# Other packages
find_package(OpenCV REQUIRED) # OpenCV
find_package(Boost REQUIRED COMPONENTS system filesystem)
find_package(OpenMP REQUIRED) # OpenMP
find_package(obindex2 REQUIRED)
if (OPENMP_FOUND)
  set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()


include_directories(include
                    external
                    ${OpenCV_INCLUDE_DIRS}
                    ${Boost_INCLUDE_DIRS}
                    # ${PROJECT_SOURCE_DIR}/../obindex2/lib/include/ # should be found by cmake
                    )

### Targets ###
# Library
add_library(lcdetector
            SHARED
            include/ibow_lcd/island.h
            include/ibow_lcd/lcdetector.h
            src/lcdetector.cc)

target_link_libraries(lcdetector
                      ${OpenCV_LIBRARIES}
                      ${Boost_LIBRARIES}
                    #   ${PROJECT_SOURCE_DIR}/../obindex2/build/lib/libobindex2.so  # should be found by cmake
                      obindex2::obindex2
                      )




set_target_properties(lcdetector
    PROPERTIES PUBLIC_HEADER
    "include/ibow_lcd/island.h;include/ibow_lcd/lcdetector.h"
    )


install(TARGETS lcdetector
    EXPORT lcdetectorConfig
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    # PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/lcdetector
    PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/ibow_lcd
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    )

export(TARGETS
    lcdetector
    NAMESPACE lcdetector::
    FILE "${CMAKE_CURRENT_BINARY_DIR}/lcdetectorConfig.cmake"
)

install(EXPORT
    lcdetectorConfig
    DESTINATION "${CMAKE_INSTALL_LIBDIR}/lcdetector/cmake"
    NAMESPACE lcdetector::
)
