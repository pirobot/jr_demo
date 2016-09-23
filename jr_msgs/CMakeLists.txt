cmake_minimum_required(VERSION 2.8.3)
project(jr_msgs)

find_package(catkin REQUIRED COMPONENTS std_msgs geometry_msgs gps_common ar_track_alvar_msgs message_generation)

add_message_files(FILES
                  Location.msg
                 )

add_service_files(FILES
                  GotoLocation.srv
                  SayPhrase.srv
                 )

generate_messages(   
	DEPENDENCIES  
	std_msgs
	geometry_msgs
	gps_common
	ar_track_alvar_msgs
)  

catkin_package(CATKIN_DEPENDS message_runtime std_msgs)
