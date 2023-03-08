// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;

public class BlinkinLEDs extends SubsystemBase {
  private Spark leds;
  private PowerDistribution pdh;

  /** Creates a new BlinkinLEDs. */
  public BlinkinLEDs() {
    leds = new Spark(LEDConstants.ledPort);
    pdh = new PowerDistribution();

    setBlack();
  }

  public void setYellow() {
    pdh.setSwitchableChannel(true);
    leds.set(0.69);
  }

  public boolean getYellow() {
    return leds.get() == 0.69;
  }

  public void setPurple() {
    pdh.setSwitchableChannel(true);
    leds.set(0.91);
  }

  public boolean getPurple() {
    return leds.get() == 0.91;
  }

  public void setGreen() {
    pdh.setSwitchableChannel(true);
    leds.set(-0.05);
  }

  public void setBlack() {
    pdh.setSwitchableChannel(true);
    leds.set(0.99);
  }

  public void turnOff() {
    pdh.setSwitchableChannel(false);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
