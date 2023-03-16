// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.Constants.BalancingConstants;
import frc.robot.subsystems.Swerve;

public class BalanceChargeStationInverse extends CommandBase {
  private boolean isContinuous;
  private PIDController pid;
  private Swerve dt;
  private double calc;
  private double stationOffset = 0;
  private double sensitivity = BalancingConstants.kSensitivity;
  private boolean isOffset = false;

  /** Creates a new BalanceChargeStationInverse. */
  public BalanceChargeStationInverse(Swerve dt, boolean isContinuous) {
    pid = new PIDController(BalancingConstants.kP, BalancingConstants.kI, BalancingConstants.kD);
    pid.setTolerance(BalancingConstants.kToleranceDegrees);
    this.dt = dt;
    this.isContinuous = isContinuous;
    pid.setSetpoint(BalancingConstants.kSetpoint);
  }

  public double throttle(double x){
    if (Math.abs(x) > 5){
      return Math.signum(x);
    }
    return x;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isOffset = false;
    stationOffset = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putString("ChargeStation", "balancing");
    calc = pid.calculate(dt.getPitch());
    if (!isOffset){
    dt.drive(
      new Translation2d(
        -throttle(calc/sensitivity)*Constants.Swerve.maxSpeed,
        0
      ),
      0,
      false,
      true
      );
    }
    SmartDashboard.putNumber("charge error", calc);
    SmartDashboard.putBoolean(" charge Setpoint", pid.atSetpoint());    
    SmartDashboard.putNumber("charge output", throttle(calc/sensitivity)*Constants.Swerve.maxSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    dt.setBrakeMode(true);
    dt.drive(new Translation2d(0, 0), 0, true, false);  
    if(!interrupted){
      SmartDashboard.putString("ChargeStation", "balance Finished");
    }else{
      SmartDashboard.putString("ChargeStation", "balance interrupted");
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(!isContinuous){//if we are not in continuous mode    
      return pid.atSetpoint();//then end the command when the robot is balanced
    }else{ 
      return false; //otherwise the command will run indefinetly
    }
  }
}
