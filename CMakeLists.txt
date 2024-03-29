cmake_minimum_required(VERSION 3.5)
project(syt_hmi)

# Default to C99
if (NOT CMAKE_C_STANDARD)
    set(CMAKE_C_STANDARD 99)
endif ()

# Default to C++14
if (NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
endif ()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED) #

find_package(Qt5 COMPONENTS Widgets Core REQUIRED)
find_package(syt_common REQUIRED)
find_package(syt_msgs REQUIRED)
find_package(OpenCV REQUIRED)


set(THIS_PACKAGE_INCLUDE_DEPENDS
        rclcpp
        rclcpp_components
        )

# Note that ament_target_dependencies() in algo cannot export Boost::thread to downstream. As a result, qt_example_node, as a downstream of libalgo.so, needs to explicitly find_package() for Boost again so that qt_example_node is able to link to Boost::thread.
set(CMAKE_INCLUDE_CURRENT_DIR ON)
file(GLOB PRO_FORM_DIR RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} ui/*.ui)
file(GLOB PRO_RESOURCES_DIR RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} resource/*.qrc)
file(GLOB_RECURSE PRO_INCLUDE_DIR RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS include/*)
file(GLOB_RECURSE PRO_SOURCES_DIR RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS src/*)

#搜索头文件的位置
include_directories(
        include/
        ${CMAKE_CURRENT_BINARY_DIR}
        ${rclcpp_INCLUDE_DIRS}
        ${Qt5Core_INCLUDE_DIRS}
        ${Qt5Widgets_INCLUDE_DIRS}
)

#qt头文件moc文件resource文件处理
qt5_wrap_ui(QT_UI_HPP ${PRO_FORM_DIR})
qt5_wrap_cpp(QT_MOC_HPP ${PRO_INCLUDE_DIR})
qt5_add_resources(QT_RESOURCES_CPP ${PRO_RESOURCES_DIR})
add_executable(${PROJECT_NAME}
        ${PRO_SOURCES_DIR}
        ${QT_RESOURCES_CPP}
        ${PRO_INCLUDE_DIR}
        ${QT_MOC_HPP}
        ${PRO_FORM_DIR}
        )

# Note that ament_target_dependencies() in algo cannot export Boost::thread to downstream. As a result, qt_example_node, as a downstream of libalgo.so, needs to explicitly ament_target_dependencies() for Boost again so that qt_example_node is able to link to Boost::thread.
ament_target_dependencies(${PROJECT_NAME}
        rclcpp
        std_srvs
        syt_common syt_msgs)

target_link_libraries(${PROJECT_NAME}
        Qt5::Widgets
        Qt5::Core
        ${OpenCV_LIBS}
        )

install(TARGETS ${PROJECT_NAME}
        DESTINATION lib/${PROJECT_NAME}
        )

# exprot package
ament_export_include_directories(
        include
)
ament_export_libraries(
        ${PROJECT_NAME}
)

ament_package()

