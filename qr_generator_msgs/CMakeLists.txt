cmake_minimum_required(VERSION 2.8.3)
project(qr_generator_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  sensor_msgs)

add_service_files(
    FILES
    GetQR.srv
    )

# Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES sensor_msgs)

catkin_package(
  CATKIN_DEPENDS message_runtime
  )
