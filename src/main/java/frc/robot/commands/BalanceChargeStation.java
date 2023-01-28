// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.BalancingConstants;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.LEDLightControl;
import frc.robot.Constants.ButtonHashtable;

public class BalanceChargeStation extends CommandBase {
  private boolean isContinuous = true; //if false, the command will end after the robot is balanced
  
  private LEDLightControl led = new LEDLightControl();
  private SetColor colors = new SetColor(led);

  private PIDController pid; // see Ramsete.java for info on PIDControllers
  private Swerve dt;
  double calc;
  double stationOffset = 0;

  private ButtonHashtable bh = new ButtonHashtable();
  

  /** Creates a new BalanceChargeStation. */
  public BalanceChargeStation(Swerve dt) {
    pid = new PIDController(BalancingConstants.kP, BalancingConstants.kI, BalancingConstants.kD);
    pid.setTolerance(BalancingConstants.kToleranceDegrees);
    this.dt = dt;
    pid.setSetpoint(BalancingConstants.kSetpoint);
  }

  /**@param stationOffset the heading that we need to be at to be aligned with the charge station */
  public BalanceChargeStation(Swerve dt, double stationOffset) {
    pid = new PIDController(BalancingConstants.kP, BalancingConstants.kI, BalancingConstants.kD);
    pid.setTolerance(BalancingConstants.kToleranceDegrees);
    this.dt = dt;
    this.stationOffset = stationOffset%360;
    pid.setSetpoint(BalancingConstants.kSetpoint);
  }

  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  dt.drive(new Translation2d(0, 0), stationOffset, true, false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //start pid loop
    //devide to reduce the sensitivity
    //if pitch within 5(?) degrees of 0, stop pid loop
    calc = -pid.calculate(dt.getPitch());
    //for every degree of pitch, drive forward 1/15 of a meter
    //dt.drive(new Translation2d(calc/BalancingConstants.sensitivity, 0), 0, true, false);
    
    dt.driveTank(calc/6);

    SmartDashboard.putNumber("error", calc);
    SmartDashboard.putBoolean("isAtSetpoint", pid.atSetpoint());

    if (!pid.atSetpoint()) {
      //0.61 is red
      colors.setColor(bh.buttons.get("Red"));
    } else {
      //0.73 is lime (green)
      colors.setColor(bh.buttons.get("Red"));
    }
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    dt.setBrakeMode(true);
    dt.drive(new Translation2d(0, 0), stationOffset, true, false);
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