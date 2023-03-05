// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;



import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.BalancingConstants;
import frc.robot.subsystems.Swerve;
//import frc.robot.subsystems.LEDLightControl;
//import frc.robot.Constants.ButtonHashtable;

public class BalanceChargeStation extends CommandBase {
  private boolean isContinuous; //if false, the command will end after the robot is balanced
  
  // private LEDLightControl led = new LEDLightControl();
  // private SetColor colors = new SetColor(led);

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
  double sensitivity = BalancingConstants.kSensitivity;
  boolean isOffset = false;

  //private ButtonHashtable bh = new ButtonHashtable();
  

  /** Creates a new BalanceChargeStation. */
  public BalanceChargeStation(Swerve dt, boolean isContinuous) {
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
        throttle(calc/sensitivity)*Constants.Swerve.maxSpeed,
        0
      ),
      0,
      true,
      true
      );
    }
    SmartDashboard.putNumber("charge error", calc);
    SmartDashboard.putBoolean(" charge Setpoint", pid.atSetpoint());    
    SmartDashboard.putNumber("charge output", throttle(calc/sensitivity)*Constants.Swerve.maxSpeed);
    
/* 
    if (!pid.atSetpoint()) {
      //0.61 is red
      colors.setColor(bh.buttons.get("Red"));
    } else {
      //0.73 is lime (green)
      colors.setColor(bh.buttons.get("Red")); //shouldnt this be lime?
    }
  
  */}
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