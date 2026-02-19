cmake_minimum_required(VERSION 3.16)
project(bartending_cobot_bringup)

set(bartending_cobot_bringup_MAJOR_VERSION 0)
set(bartending_cobot_bringup_MINOR_VERSION 0)
set(bartending_cobot_bringup_PATCH_VERSION 0)
set(bartending_cobot_bringup_VERSION
  ${bartending_cobot_bringup_MAJOR_VERSION}.${bartending_cobot_bringup_MINOR_VERSION}.${bartending_cobot_bringup_PATCH_VERSION})

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
