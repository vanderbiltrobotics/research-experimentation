// User Configuration object, formatted as JSON
// Can be any valid (even nested) JSON object,
// Accessible from within component code as:
//   config['User Configuration']
{
  "moveAxis": 1, // Moving left joystick up (+) and down (-) changes linear velocity
  "rotateAxis": 3, // Moving left joystick left (+) and right (-) changes rotational velocity
  "augerLeadAxis" : 7, // Moving right joystick up (+) and down (-) changes speed of lead screw that moves auger into and out of ground
  "augerDigDownAxis" : 5, // Moving right joystick left (+) and right (-) changes speed of digging (speed that auger spins)
  "augerDigUpAxis": 2,
  "extendAugerButton": 5, // Pressing 'Y' button extends auger out
  "retractAugerButton": 4,
  "conveyorEjectButton" : 3, // Pressing up on D-pad causes conveyor to turn (hold to keep turning)
  "conveyorReverseButton": 1,
  "autonButton" : 8, // Pressing 'A' publishes "AUTON" robot state msg
  "teleopButton" : 7, // Pressing 'X' publishes "TELEOP" robot state msg
  "disableButton" : 6, // Pressing 'B' publishes "DISABLED" robot state msg
  "turboButton" : 2
}