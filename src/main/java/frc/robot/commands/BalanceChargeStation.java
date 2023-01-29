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

  private PIDController pid; 
  // for in depth info on PID controllers, see https://docs.wpilib.org/en/stable/docs/software/advanced-controls/controllers/pidcontroller.html
  /*basic rundown:
   * setpoint is the value we want to reach
   * error is the difference between the setpoint and the current value
   * the PID controller calculates the error (difference between current value and setpoint) 
   * and uses it to calculate the output
   * 
   *  gains are constants that are used to calculate the output and need to be tuned to 
   *  your specific application
   *  gains are multiplied by the P,I, and D terms to calculate the output
   * 
   *  steady state error: when the PID never reaches the setpoint 
   * 
   *  P term: the error
   *  I term: the sum of all the errors, used to correct for steady state error
   *  D term: the rate of change of the error, used to correct for overshoot
   * 
   * you dont need to use all 3 terms, but you should always use the P term
   */
  
  private Swerve dt;
  double calc;
  double stationOffset = 0;
  double sensitivity = 10;

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
  //orient wheels to be parallel with the charge station
  dt.drive(new Translation2d(0, 0), stationOffset, true, false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //start pid loop
    //devide to reduce the sensitivity
    //if pitch within 5(?) degrees of 0, stop pid loop
    calc = -pid.calculate(dt.getPitch());
    // if calc = sensitivity, drive at 100% speed, if calc = 0, drive at 0% speed, etc
    dt.driveTank(calc/sensitivity);

    SmartDashboard.putNumber("error", calc);
    SmartDashboard.putBoolean("isAtSetpoint", pid.atSetpoint());    


    if (!pid.atSetpoint()) {
      //0.61 is red
      colors.setColor(bh.buttons.get("Red"));
    } else {
      //0.73 is lime (green)
      colors.setColor(bh.buttons.get("Red")); //shouldnt this be lime?
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