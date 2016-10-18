project("riegl")

find_package(catkin REQUIRED COMPONENTS gencpp std_msgs message_generation)

add_message_files(
  FILES
    RieglStatus.msg
    RieglTime.msg
)

generate_messages(DEPENDENCIES std_msgs)

