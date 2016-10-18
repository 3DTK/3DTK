project("volksbotRT")

find_package(catkin REQUIRED COMPONENTS gencpp std_msgs message_generation)

add_message_files(
  FILES
    ticks.msg
    pose2d.msg
)

generate_messages(DEPENDENCIES std_msgs)

