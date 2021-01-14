cmake_minimum_required(VERSION 2.8.3)
project(student_proect)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(OpenCV REQUIRED )
find_package(project_interface REQUIRED )

## Specify additional locations of header files
include_directories(
 include 
 ${OpenCV_INCLUDE_DIRS}
 ${project_interface_INCLUDE_DIRS}
)

##---------------
add_executable(calibrate
  src/camera_calibration.cpp
  )
##--------------

## CREATE EXECUTABLE LIBRARY
add_library(student SHARED
   src/student_interface.cpp
)

##-----------
add_library(dubins SHARED
   src/dubins.cpp
)
add_library(polyclipping SHARED
   src/clipper/clipper.cpp
)
##----------

target_link_libraries(student
  ${OpenCV_LIBRARIES} 
  dubins
  polyclipping	
)

add_library(match_template SHARED
   src/match_template.cpp
)

target_link_libraries(match_template
  ${OpenCV_LIBRARIES}  
)

add_library(ocr SHARED
   src/ocr.cpp
)

target_link_libraries(ocr
  tesseract
  ${OpenCV_LIBRARIES}  
)
#----
target_link_libraries(calibrate
  ${OpenCV_LIBRARIES} 
  )
