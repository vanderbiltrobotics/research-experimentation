if(robotState == TELEOP) {
  //std::cout << "Called Operation in JoystickSubscriber" << std::endl;
  sensor_msgs::Joy msg = lastMessage;
  
  // Joystick Axes
  setDriveSpeed(msg.axes[moveAxis], msg.axes[rotateAxis]);
  setLeadSpeed(msg.axes[augerLeadAxis]);
  // Adjust trigger range to account for 1 when not pressed
  double augerDigSpeed = 0;
  if(msg.axes[augerDigDownAxis]<1){
  	setAugerSpeed((msg.axes[augerDigDownAxis]-1) / -2.0);
  }else if (msg.axes[augerDigUpAxis]<1){
    setAugerSpeed((msg.axes[augerDigUpAxis]-1) / 2.0);
  }else{
  	setAugerSpeed(0.0); 
  }
  
  // Joystick Buttons
  if(msg.buttons[extendAugerButton]){
    extendAuger(1.0);
  }else if(msg.buttons[retractAugerButton]){
  	extendAuger(-1.0); 
  }else{
  	extendAuger(0.0); 
  }
  if(msg.buttons[conveyorEjectButton]){
  	setConveyorSpeed(1.0); 
  }else if(msg.buttons[conveyorReverseButton]){
  	setConveyorSpeed(-1.0);
  }else{
  	setConveyorSpeed(0.0); 
  }
  if(msg.buttons[turboButton]) {
  	setTurboState(true);
  }else{
  	setTurboState(false);
  }
}else{
  setTurboState(false);
}