cmake_minimum_required(VERSION 2.8.3)
project(adi_pico_driver)

#add_compile_options(-std=c++11)
include("../cmake_modules/CMakeOpt.cmake")

find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs std_srvs roslint)
find_package(Boost REQUIRED COMPONENTS system)

catkin_package(CATKIN_DEPENDS roscpp sensor_msgs std_srvs)

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

add_executable(adis16470_node src/main.cpp)
target_link_libraries(adis16470_node ${catkin_LIBRARIES} ${Boost_LIBRARIES})

install(DIRECTORY launch urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
  )

if(CATKIN_ENABLE_TESTING)
  find_package(catkin REQUIRED COMPONENTS rostest roslaunch)
  file(GLOB LAUNCH_FILES launch/*.launch)
  foreach(LAUNCH_FILE ${LAUNCH_FILES})
    message(STATUS "Testing ${LAUNCH_FILE}")
    roslaunch_add_file_check(${LAUNCH_FILE})
  endforeach()
  roslaunch_add_file_check(launch/adis16470.launch with_rviz:=true with_plot:=true publish_temperature:=true)
endif()

roslint_cpp()

# debbuild
# catkin build minas_control --make-args debbuild
# ROS_PARALLEL_JOBS='' catkin_make --pkg ${PROJECT_NAME} -- debbuild
add_custom_target(debbuild_${PROJECT_NAME}
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
  COMMAND bloom-generate rosdebian --os-name ubuntu --os-version xenial --ros-distro kinetic
  COMMAND fakeroot debian/rules binary
  COMMAND find debian obj-x86_64-linux-gnu -type f -not -name postinst -print0 | xargs -0 rm --
  COMMAND find debian obj-x86_64-linux-gnu -type d -empty -delete
)
