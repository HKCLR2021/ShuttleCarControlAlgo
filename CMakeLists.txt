cmake_minimum_required(VERSION 3.5)
project(profiled_pid_controller)

# Default to C99
if (NOT CMAKE_C_STANDARD)
    set(CMAKE_C_STANDARD 99)
endif ()

# Default to C++14
if (NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif ()

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

# looking for spdlog
if (NOT TARGET spdlog)
    find_package(spdlog REQUIRED)
endif ()

add_library(${PROJECT_NAME} STATIC
        src/TrapezoidProfile.cpp
        src/PIDController.cpp)

target_include_directories(${PROJECT_NAME} PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include/frc)

target_include_directories(${PROJECT_NAME} PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include)

target_link_libraries(${PROJECT_NAME} spdlog::spdlog)

# driver test files
add_executable(Tester test/tester.cpp)
target_include_directories(Tester PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(Tester ${PROJECT_NAME})

set(HEADER_FILES
        include/frc/PIDController.h
        include/frc/ProfiledPIDController.h
        include/frc/TrapezoidProfile.h
)

set_target_properties(${PROJECT_NAME}
        PROPERTIES
        FRAMEWORK ON
        SOVERSION 0
        VERSION 0.1.0
        PUBLIC_HEADER "${HEADER_FILES}"
        # PRIVATE_HEADER "${PRIVATE_HEADER_FILES}"
        LIBRARY_OUTPUT_DIRECTORY "lib"
        OUTPUT_NAME ${PROJECT_NAME}
)

# set(CMAKE_INSTALL_PREFIX ${CMAKE_CURRENT_SOURCE_DIR}/install)

install(TARGETS ${PROJECT_NAME}
        LIBRARY DESTINATION lib
        PUBLIC_HEADER DESTINATION include
)
