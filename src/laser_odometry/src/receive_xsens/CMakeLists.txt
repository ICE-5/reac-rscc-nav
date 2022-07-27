cmake_minimum_required(VERSION 2.8.3)
project(receive_xsens)

SET(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
set(CMAKE_BUILD_TYPE "Debug")

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
)

find_package(Boost REQUIRED system thread)
find_package(OpenCV REQUIRED)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  CATKIN_DEPENDS
  roscpp
  std_msgs
  sensor_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  /usr/local/xsens/include # Location when using 'make system_install'
  /usr/local/include # Location when using 'make system_install'
  /usr/include       # More usual location (e.g. when installing using a package)
)

## Specify additional locations for the xsens library
link_directories(
  /usr/local/xsens/lib # Location when using 'make system_install'
  /usr/local/lib # Location when using 'make system_install'
  /usr/lib       # More usual location (e.g. when installing using a package)
)

## Declare executables
add_executable(receive_xsens src/main.cpp src/conio.c src/deviceclass.cpp src/udp.cpp src/timeoutserial.cpp src/xcommunication/enumerateusbdevices.cpp src/xcommunication/legacydatapacket.cpp src/xcommunication/protocolhandler.cpp src/xcommunication/packetfixeddata.cpp src/xcommunication/usbinterface.cpp src/xcommunication/mtwsdidata.cpp src/xcommunication/iointerface.cpp src/xcommunication/serialinterface.cpp src/xcommunication/int_xsdatapacket.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(receive_xsens
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  xsensdeviceapi
  xstypes
)

install(TARGETS receive_xsens
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES receive_xsens.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY chrony/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/chrony
)
