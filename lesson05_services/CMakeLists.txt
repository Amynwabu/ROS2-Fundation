cmake_minimum_required(VERSION 3.8)
project(my_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# ADD THIS LINE
find_package(rosidl_default_generators REQUIRED)

# ADD THIS BLOCK
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/TakeOrder.srv"
)

ament_package()
