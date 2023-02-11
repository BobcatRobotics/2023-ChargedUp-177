// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;

public class Elevator extends SubsystemBase {
  private WPI_TalonFX elevatorMotor;

  private DigitalInput topLimit;
  private DigitalInput bottomLimit;

  /** Creates a new Elevator. */
  public Elevator() {
    elevatorMotor = new WPI_TalonFX(ElevatorConstants.elevatorMotorPort);
    topLimit = new DigitalInput(ElevatorConstants.topLimitPort);
    bottomLimit = new DigitalInput(ElevatorConstants.bottomLimitPort);

    elevatorMotor.configFactoryDefault();
    elevatorMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 20);
    elevatorMotor.setSensorPhase(true);
    elevatorMotor.configNominalOutputForward(0, 20);
    elevatorMotor.configNominalOutputReverse(0, 20);
    elevatorMotor.configPeakOutputForward(1, 20);
    elevatorMotor.configPeakOutputReverse(-1, 20);
    elevatorMotor.configAllowableClosedloopError(0, 0, 20);
    elevatorMotor.config_kF(0, 0, 20);
    elevatorMotor.config_kP(0, 0.1, 20);
    elevatorMotor.config_kI(0, 0, 20);
    elevatorMotor.config_kD(0, 0.5, 20);

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
    if (state == 0) {
      elevatorMotor.set(ControlMode.Position, ElevatorConstants.pos0);
    } else if (state == 1) {
      elevatorMotor.set(ControlMode.Position, ElevatorConstants.pos1);
    } else {
      elevatorMotor.set(ControlMode.Position, ElevatorConstants.pos2);
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
