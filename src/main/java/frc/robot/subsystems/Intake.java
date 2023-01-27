// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  private WPI_TalonFX motor; 
  private Solenoid grabber;

  /** Creates a new Intake. */
  public Intake() {
   motor = new WPI_TalonFX(Constants.intakeMotorID);
   grabber = new Solenoid(null, Constants.intakeSolenoidID);
  }

  public void runIntakeIn(){
    motor.set(ControlMode.PercentOutput, 0.7);
  }
  public void runIntakeOut(){
    motor.set(ControlMode.PercentOutput, -0.4);
  }
  public void openGrabber(){
    grabber.set(false);
  }
  public void closeGrabber(){
    grabber.set(true);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
  }
}
