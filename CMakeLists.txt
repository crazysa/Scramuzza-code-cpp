cmake_minimum_required(VERSION 3.12)
project(UndistortFtheta)

# Find the OpenCV package
find_package(OpenCV REQUIRED PATHS -I/usr/local/include/opencv4)

# Add the executable
add_executable(UndistortFtheta UndistortFthetaImage.cpp ocam_functions.cpp)

# Include OpenCV headers
target_include_directories(UndistortFtheta PRIVATE ${OpenCV_INCLUDE_DIRS})

# Link the OpenCV library to the executable
target_link_libraries(UndistortFtheta PRIVATE ${OpenCV_LIBS})