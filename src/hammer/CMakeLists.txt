cmake_minimum_required(VERSION 3.8)
project(hammer)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)


set(NODES
  hammer/hammer.py
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
ament_package()
