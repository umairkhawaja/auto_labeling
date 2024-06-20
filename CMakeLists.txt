cmake_minimum_required(VERSION 3.1)

project(auto_labeling)

# Set flags - you might not need this line if you've already set it above
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")

# Define the installation directory
set(CMAKE_INSTALL_PREFIX ${PROJECT_SOURCE_DIR})
set(INSTALL_DIR ${CMAKE_INSTALL_PREFIX}/bin)

# Add your source code directory
add_subdirectory(src)

# Install the executable to the specified installation directory
install(TARGETS ${PROJECT_NAME} DESTINATION ${INSTALL_DIR})
