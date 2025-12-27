cmake_minimum_required(VERSION 3.16)
project(manipulator_description)

set(manipulator_description_MAJOR_VERSION 0)
set(manipulator_description_MINOR_VERSION 0)
set(manipulator_description_PATCH_VERSION 0)
set(manipulator_description_VERSION
  ${manipulator_description_MAJOR_VERSION}.${manipulator_description_MINOR_VERSION}.${manipulator_description_PATCH_VERSION})

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
