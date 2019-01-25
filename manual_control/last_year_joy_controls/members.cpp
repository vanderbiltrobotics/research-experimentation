// Joystick axes -- See User Configuration for joystick/button assignments
int moveAxis;
int rotateAxis;
int augerLeadAxis;
int augerDigDownAxis;
int augerDigUpAxis;

// Joystick buttons
int extendAugerButton;
int retractAugerButton;
int conveyorEjectButton;
int conveyorReverseButton;
int autonButton;
int teleopButton;
int disableButton;
int turboButton;

sensor_msgs::Joy lastMessage;

const std::string DISABLED = "DISABLED";
const std::string AUTON = "AUTON";
const std::string TELEOP = "TELEOP";

std::string robotState;

void setDriveSpeed(double linear, double rotational){
	geometry_msgs::Twist msg;
	msg.linear.x = linear;
  	msg.angular.z = rotational;
  	JoystickDrivetrainPublisher.publish(msg);
}

void setConveyorSpeed(double speed){
	std_msgs::Float64 msg;
  	msg.data = speed;
    JoystickConveyorPublisher.publish(msg);
}

void setAugerSpeed(double rotational) {
	std_msgs::Float64 msg;
  	msg.data = rotational;
    JoystickAugerSpinPublisher.publish(msg);
}

void setLeadSpeed(double rotational) {
	std_msgs::Float64 msg;
  	msg.data = rotational;
    JoystickAugerScrewPublisher.publish(msg);
}

void extendAuger(double power) { // 0 means stop, anything else means go
  std_msgs::Float64 msg;
  msg.data = power;
  JoystickAugerLinearPublisher.publish(msg);
}

void setTurboState(bool on) {
	std_msgs::Bool msg;
  	msg.data = on;
  	TurboPublisher.publish(msg);
}