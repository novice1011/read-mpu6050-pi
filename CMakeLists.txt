cmake_minimum_required(VERSION 3.0.0)

project(read-mpu6050-pi VERSION 0.1 LANGUAGES CXX)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)

find_package(Qt5Core)

add_executable(${PROJECT_NAME} "main.cpp")

include_directories(/home/omank/Manually_Installed/raspberry_pi_tools/wiringpi/include)

target_link_libraries(${PROJECT_NAME} Qt5::Core -L/home/omank/Manually_Installed/raspberry_pi_tools/wiringpi/lib -lwiringPi)


#/usr/local/include/
#/usr/local/lib/
