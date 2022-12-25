cmake_minimum_required(VERSION 3.1.1)
project(rover_firmware)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs)

find_package(JetsonGPIO)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}

)

add_executable(encoders encoders/encoders.cpp )
target_link_libraries(encoders ${catkin_LIBRARIES} JetsonGPIO::JetsonGPIO -lpthread -lcrypt -lm -lrt -Wno-psabi)

#add_executable(dc_motors src/dc_motors.cpp)
#target_link_libraries(dc_motors ${catkin_LIBRARIES} JetsonGPIO::JetsonGPIO -lpthread -lcrypt -lm -lrt)
