// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;

public class BlinkinLEDs extends SubsystemBase {
  private Spark leds;

  /** Creates a new BlinkinLEDs. */
  public BlinkinLEDs() {
    leds = new Spark(LEDConstants.ledPort);
  }

  public void setYellow() {
    leds.set(0.69);
  }

  public void setPurple() {
    leds.set(0.91);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
