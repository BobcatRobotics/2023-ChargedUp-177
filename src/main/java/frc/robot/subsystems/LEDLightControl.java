// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.



// idk why, but this broke all my code, so im commenting it out for now - Aiden

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


import frc.robot.Constants;

public class LEDLightControl extends SubsystemBase {
  private Spark LEDs;
  private Constants.ColorHashtable ch = new Constants.ColorHashtable(); 

  /** Creates a new LEDLightControl. */
  public LEDLightControl() {
    //The LED thingamajigalading used in the robot
    LEDs = new Spark(8);
    //black
    this.colorSet(ch.colors.get("Black"));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  
  public void colorSet(double color) {
    // solid colors: 0.57 - 0.99
    LEDs.set(color);
  }
}
