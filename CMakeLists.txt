cmake_minimum_required(VERSION 2.8)
project(RudderTest)
# Locate libraries and headers
find_library(wiringPi_LIB wiringPi)
find_package(Threads REQUIRED)

# Include headers
include_directories(${WIRINGPI_INCLUDE_DIRS}) 
include_directories("../RemoteControlTest")

#the file(GLOB...) allows for wildcard additions:
file(GLOB SOURCES "./*.cpp")
add_executable(RudderTest ${SOURCES} "../RemoteControlTest/AToD.cpp" "../RemoteControlTest/ShipLog.cpp" "../RemoteControlTest/CommandList.cpp" "../RemoteControlTest/Rudder.cpp"
"../RemoteControlTest/DiagnosticsSensor.cpp" "../RemoteControlTest/Sensor.cpp"
"../RemoteControlTest/HumiditySensor.cpp" "../RemoteControlTest/BatteryCharge.cpp"
"../RemoteControlTest/filedata.cpp" "../RemoteControlTest/Util.cpp")


# Link against libraries
target_link_libraries(RudderTest ${wiringPi_LIB} )
target_link_libraries(RudderTest ${CMAKE_THREAD_LIBS_INIT})
