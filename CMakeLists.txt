cmake_minimum_required(VERSION 2.8)
project( FTSensors )

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_COMPILER_IS_GNUCXX)
	message(ERROR "The compiler ${CMAKE_CXX_COMPILER} is not a GNU C++ compiler. Please use a GNU C++ compiler.")
endif()

if(WIN32)
	#required to suppress the boost warning
	add_definitions(-D _WIN32_WINNT=0x0501)
endif()

SET(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

find_package(Boost 1.6 REQUIRED COMPONENTS system thread atomic chrono)

include_directories(${Boost_INCLUDE_DIRS})
include_directories(include)

add_library(staticLib STATIC src/${PROJECT_NAME}.cpp)
set_target_properties(staticLib PROPERTIES OUTPUT_NAME ${PROJECT_NAME})

#add_library(sharedLib SHARED src/${PROJECT_NAME}.cpp)
#set_target_properties(sharedLib PROPERTIES OUTPUT_NAME ${PROJECT_NAME})

if(WIN32)
	target_link_libraries(staticLib ${Boost_LIBRARIES} ws2_32)
	#target_link_libraries(sharedLib ${Boost_LIBRARIES} ws2_32)
else()
	target_link_libraries(staticLib ${Boost_LIBRARIES})
	#target_link_libraries(sharedLib ${Boost_LIBRARIES})
endif()

#####################################################################
#Build test app
#####################################################################

SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/test/bin)

include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${PROJECT_SOURCE_DIR}/test/include)

link_directories(${PROJECT_SOURCE_DIR}/lib)

add_executable( ATI_NetBox_FT_Sensor ${PROJECT_SOURCE_DIR}/test/src/ATI_NetBox_FT_Sensor.cpp )

if(WIN32)
	target_link_libraries(ATI_NetBox_FT_Sensor FTSensors ${Boost_LIBRARIES} ws2_32)
else()
	target_link_libraries(ATI_NetBox_FT_Sensor FTSensors ${Boost_LIBRARIES})
endif()
