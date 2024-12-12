cmake_minimum_required(VERSION 3.8)
project(hammer)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(SRV_FILES
  srv/GetList.srv
  srv/KillRequest.srv
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${SRV_FILES}
)

set(NODES
  hammer/hammer_node.py
)

install(PROGRAMS
    ${NODES}
DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY 
    hammer/static
    hammer/templates
  DESTINATION lib/${PROJECT_NAME}
)


ament_export_dependencies(rosidl_default_runtime)
# ament_python_install_package(${PROJECT_NAME})
ament_package()

set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
include(packing)