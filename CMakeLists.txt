# cmake needs this line
cmake_minimum_required(VERSION 3.10)
# Define project name
project(hik_mindvision_camera)
# Find OpenCV, you may need to set OpenCV_DIR variable
# to the absolute path to the directory containing OpenCVConfig.cmake file
# via the command line or GUI
find_package(OpenCV REQUIRED)
# If the package has been found, several variables will
# be set, you can find the full list with descriptions
# in the OpenCVConfig.cmake file.
# Print some message showing some of them
# set "-std=c++11"
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

# Add OpenCV headers location to your include paths
# include_directories(${OpenCV_INCLUDE_DIRS})

# Declare the executable target built from your sources
add_executable(hik_camera src/hik_camera.cpp)
# Link your application with OpenCV libraries
target_include_directories(hik_camera PUBLIC hikSDK/include include)

if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
  target_link_directories(hik_camera PUBLIC hikSDK/lib/amd64)
  install(
    DIRECTORY hikSDK/lib/amd64/
    DESTINATION lib
  )
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
  target_link_directories(hik_camera PUBLIC hikSDK/lib/arm64)
  install(
    DIRECTORY hikSDK/lib/arm64/
    DESTINATION lib
  )
else()
  message(FATAL_ERROR "Unsupport host system architecture: ${CMAKE_HOST_SYSTEM_PROCESSOR}!")
endif()

target_link_libraries(hik_camera
  FormatConversion
  MediaProcess
  MvCameraControl
  MVRender
  MvUsb3vTL
  ${OpenCV_LIBS}
)

add_executable(mindvision_camera src/mindvision_camera.cpp)
target_include_directories(mindvision_camera PUBLIC mvsdk/include include)

if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
  target_link_directories(mindvision_camera PUBLIC mvsdk/lib/amd64)
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
  target_link_directories(mindvision_camera PUBLIC mvsdk/lib/arm64)
else()
  message(FATAL_ERROR "Unsupport host system architecture: ${CMAKE_HOST_SYSTEM_PROCESSOR}!")
endif()

target_link_libraries(mindvision_camera MVSDK ${OpenCV_LIBS})