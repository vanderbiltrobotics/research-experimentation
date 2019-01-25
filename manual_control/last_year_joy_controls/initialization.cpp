// Joystick axes -- See User Configuration for joystick/button assignments
moveAxis = config["User Configuration"]["moveAxis"].asInt();
rotateAxis = config["User Configuration"]["rotateAxis"].asInt();
augerLeadAxis = config["User Configuration"]["augerLeadAxis"].asInt();
augerDigDownAxis = config["User Configuration"]["augerDigDownAxis"].asInt();
augerDigUpAxis = config["User Configuration"]["augerDigUpAxis"].asInt();

// Joystick buttons
extendAugerButton = config["User Configuration"]["extendAugerButton"].asInt();
retractAugerButton = config["User Configuration"]["retractAugerButton"].asInt();
conveyorEjectButton = config["User Configuration"]["conveyorEjectButton"].asInt();
conveyorReverseButton = config["User Configuration"]["conveyorReverseButton"].asInt();
autonButton = config["User Configuration"]["autonButton"].asInt();
teleopButton = config["User Configuration"]["teleopButton"].asInt();
disableButton = config["User Configuration"]["disableButton"].asInt();
turboButton = config["User Configuration"]["turboButton"].asInt();