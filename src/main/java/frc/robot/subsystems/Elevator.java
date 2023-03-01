// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;



public class Elevator extends SubsystemBase {

  private DigitalInput topLimit;
  private DigitalInput bottomLimit;
  private WPI_TalonFX elevator_motor;


  public Elevator() {
    topLimit = new DigitalInput(Constants.toplimitSwitchcanId);
    bottomLimit = new DigitalInput(Constants.bottomlimitSwitchcanId);
    elevator_motor = new WPI_TalonFX(Constants.Elevator_motorCanID);
  }

  public boolean getTopLimit() {
    return topLimit.get();
  }

  public boolean getBottomLimit() {
    return bottomLimit.get();
  }
  
  public void stop() {
    elevator_motor.stopMotor();
  }
  public void elevatorMotorset(double voltage_amount){
    elevator_motor.setVoltage(voltage_amount);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
