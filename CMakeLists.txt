cmake_minimum_required(VERSION 3.5)
project(syt_hmi)
set(CMAKE_CXX_STANDARD 14)
# TODO
set(CMAKE_BUILD_TYPE "Debug")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED) #

find_package(Qt5 COMPONENTS Widgets Core REQUIRED)
find_package(syt_common REQUIRED)
find_package(syt_msgs REQUIRED)
#find_package(syt_lib_crypt REQUIRED)
find_package(OpenCV REQUIRED)


set(THIS_PACKAGE_INCLUDE_DEPENDS
        rclcpp
        rclcpp_components
        )

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

ament_target_dependencies(${PROJECT_NAME}
        rclcpp
        std_srvs
        syt_common
        syt_msgs
        #syt_lib_crypt
        )

target_link_libraries(${PROJECT_NAME}
        Qt5::Widgets
        Qt5::Core
        ${OpenCV_LIBS}
        )

install(TARGETS ${PROJECT_NAME}
        DESTINATION lib/${PROJECT_NAME}
        )

install(DIRECTORY launch
	DESTINATION share/${PROJECT_NAME}/
	)

# exprot package
ament_export_include_directories(
        include
)
ament_export_libraries(
        ${PROJECT_NAME}
)

ament_package()

