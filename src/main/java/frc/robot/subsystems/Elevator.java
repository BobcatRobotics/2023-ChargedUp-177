// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;

public class Elevator extends SubsystemBase {
  private TalonFX elevatorMotor;

  private DigitalInput topLimit;
  private DigitalInput bottomLimit;

  private PIDController elevatorController;

  /** Creates a new Elevator. */
  public Elevator() {
    elevatorMotor = new TalonFX(ElevatorConstants.elevatorMotorPort);
    elevatorMotor.configFactoryDefault();
    elevatorMotor.configIntegratedSensorOffset(elevatorMotor.getSelectedSensorPosition());

    topLimit = new DigitalInput(ElevatorConstants.topLimitPort);
    bottomLimit = new DigitalInput(ElevatorConstants.bottomLimitPort);

    elevatorController = new PIDController(0.002, 0, 0);
  }

  public void elevate(double speed) {
    elevatorMotor.set(ControlMode.PercentOutput, speed);
  }

  // public boolean getTopLimits() {
  //   return !topLimit.get();
  // }

  // public boolean getBottomLimits() {
  //   return !bottomLimit.get();
  // }

  public boolean isAtTopLimit() {
    return elevatorMotor.getSelectedSensorPosition() >= Constants.ElevatorConstants.topLimit;
  }

  public boolean isAtBottomLimit() {
    return elevatorMotor.getSelectedSensorPosition() <= Constants.ElevatorConstants.bottomLimit;
  }

  public void setState(int state) {
    double output;
    if (state == 0) {
      output = elevatorController.calculate(elevatorMotor.getSelectedSensorPosition(), Constants.ElevatorConstants.pos0);
      elevatorMotor.set(ControlMode.Position, output);
    } else if (state == 1) {
      output = elevatorController.calculate(elevatorMotor.getSelectedSensorPosition(), Constants.ElevatorConstants.pos1);
      elevatorMotor.set(ControlMode.Position, output);
    } else {
      output = elevatorController.calculate(elevatorMotor.getSelectedSensorPosition(), Constants.ElevatorConstants.pos2);
      elevatorMotor.set(ControlMode.Position, output);
    }
  }

  public int getState() {
    double pos = elevatorMotor.getSelectedSensorPosition();
    if (pos <= 256) pos = 0;
    return (int) Math.ceil(pos/4096);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
