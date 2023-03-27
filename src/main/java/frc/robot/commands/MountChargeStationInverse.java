// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;

public class MountChargeStationInverse extends CommandBase {

  private Swerve swerve;
  private double pitch;
  private double stage1Threshold = -10;
  private double stage2Threshold = -15;
  private int stage = 1;
  private boolean isRed;
  private boolean finished = false;
  private double xSpeed = 2.75;

  /** Creates a new MountChargeStationInverse. */
  public MountChargeStationInverse(Swerve drivetrain) {
    swerve = drivetrain;
    addRequirements(drivetrain);
    this.isRed = isRed;
    stage1Threshold = -10;
    //stage2Threshold = 16; - akash says max tilt is 15 degrees
    stage2Threshold = -15;
    stage = 1;
    finished = false;
    // xSpeed = -1.75; - tread worn out currently
    //xSpeed = -2.25; - worked but too slow
    xSpeed = 2.75;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    stage1Threshold = -10;
    //stage2Threshold = 16; - akash says max tilt is 15 degrees
    stage2Threshold = -15;
    stage = 1;
    finished = false;
    // xSpeed = -1.75; - tread worn out currently
    xSpeed = 2.75;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    swerve.resetModulesToAbsolute();
    pitch = swerve.getPitch();
    swerve.drive(new Translation2d(xSpeed, 0), /*may need to be changed*/
     0, false, true);
    
    
      if((pitch < stage1Threshold)){
        SmartDashboard.putString("ChargeStation", "stage 1: " + pitch);
        stage = 2;
        xSpeed = 1.25;
    
      }
      else if(stage == 2 && (pitch > stage2Threshold)){
        SmartDashboard.putString("ChargeStation", "stage 2: " + pitch);
        finished = true;
      }
      
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerve.drive(new Translation2d(0, 0), 0, true, true);
    
    SmartDashboard.putString("ChargeStation", "Finished: " + pitch);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}
