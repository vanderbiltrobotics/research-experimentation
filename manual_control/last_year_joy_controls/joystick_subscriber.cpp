// Save message to be used in Timer
lastMessage = *received_data;

// If one of these buttons was pressed, send message through immediately
if(lastMessage.buttons[disableButton]) {
  std_msgs::String msg;
  msg.data = DISABLED;
  RobotStatePublisher.publish(msg);
} else if(lastMessage.buttons[teleopButton]) {
  std_msgs::String msg;
  msg.data = TELEOP;
  RobotStatePublisher.publish(msg);
} else if(lastMessage.buttons[autonButton]) {
  std_msgs::String msg;
  msg.data = AUTON;
  RobotStatePublisher.publish(msg);
}