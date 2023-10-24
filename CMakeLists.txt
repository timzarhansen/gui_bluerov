cmake_minimum_required(VERSION 3.8)
project(gui_bluerov)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(commonbluerovmsg REQUIRED)
#find_package(px4_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3)
find_package(tf2 REQUIRED)

find_package(cv_bridge REQUIRED)
find_package(waterlinked_a50 REQUIRED)



# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)
find_package(Qt5 COMPONENTS Core Gui  Widgets Gamepad REQUIRED)
#find_package(OpenCV REQUIRED)
find_package(Qt5PrintSupport REQUIRED)

add_executable(guibluerov2 src/gui/guiBlueROV2.cpp src/gui/mainwindow.h src/gui/mainwindow.cpp src/gui/rosHandlerGui.cpp src/gui/rosHandlerGui.h src/gui/qcustomplot.h src/gui/qcustomplot.cpp)
ament_target_dependencies(guibluerov2 rclcpp std_msgs sensor_msgs commonbluerovmsg tf2 Eigen3 cv_bridge waterlinked_a50)
target_link_libraries(guibluerov2 Qt5::Core Qt5::Gui Qt5::Widgets Qt5::PrintSupport Qt5::Gamepad)



install(TARGETS
        guibluerov2
        DESTINATION lib/${PROJECT_NAME}
)


ament_package()
