// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;

public class BlinkinLEDs extends SubsystemBase {
  private Spark leds;
  private PowerDistribution pdh;
  boolean isPurple;
  boolean init; // have the lights been turned on

  /** Creates a new BlinkinLEDs. */ //color 1 = yellow
  public BlinkinLEDs() {
    leds = new Spark(LEDConstants.ledPort);
    pdh = new PowerDistribution();
    init = false;
    setBlack();
  }


  public void setYellow() {
    pdh.setSwitchableChannel(true);
      leds.set(0.69);
      
      isPurple = false;
      init = true;
  }

  public boolean getYellow() {
    return leds.get() < 0.69+0.1 && leds.get() > 0.69 + 0.1;
  }
  // 0.27 red blink
  // 0.07 purple blink
  public void blinkPurple(){
    pdh.setSwitchableChannel(true);
    leds.set(0.15);
  }
  public void blinkYellow(){
    pdh.setSwitchableChannel(true);
    leds.set(-0.07);
  }
  public void setPurple() {
    pdh.setSwitchableChannel(true);
    leds.set(0.91);
    isPurple = true;
    init = true;
  }

  public boolean isPurple(){
    return isPurple && init;
  }

  

  public boolean getPurple() {
    return (leds.get() < 0.91+0.1) && (leds.get() > 0.91 - 0.1);
  }
  public boolean getBlack() {
    return (leds.get() < 0.99+0.1) && (leds.get() > 0.99 - 0.1);
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
    SmartDashboard.putNumber("led state",leds.get());
  }
}
