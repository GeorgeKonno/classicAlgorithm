cmake_minimum_required(VERSION 2.8.3)

project(ransacAlgorithm)

find_package(OpenCV 3.3 REQUIRED)
#message(${OpenCV_FOUND})
#message(${OpenCV_LIBRARIES})

add_executable(${PROJECT_NAME} ransacAlgorithm.cpp)

target_link_libraries(${PROJECT_NAME}
  ${OpenCV_LIBRARIES}
)

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
     message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()
