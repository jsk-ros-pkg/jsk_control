cmake_minimum_required(VERSION 2.8.3)
project(contact_states_observer)

find_package(catkin REQUIRED message_generation std_msgs)

add_message_files(
  FILES GraspReferenceData.msg GraspStates.msg OREContactStates.msg GraspState.msg OREContactState.msg
)
generate_messages(
  DEPENDENCIES std_msgs
)
catkin_package(
)
