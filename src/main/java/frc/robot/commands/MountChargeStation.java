// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;

public class MountChargeStation extends CommandBase {
  
  // stage 1 -> drive forward until pitch increases beyond a certain threshold
  // stage 2 -> drive forward until pitch decreases below a certain threshold
  
  private Swerve swerve;
  private double pitch;
  private double stage1Threshold = 5;
  private double stage2Threshold = 3;
  private int stage = 1;
  private boolean isRed;
  private boolean finished = false;

  
  public MountChargeStation(Swerve drivetrain, boolean isRed) {
    swerve = drivetrain;
    addRequirements(drivetrain);
    this.isRed = isRed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    finished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    swerve.resetModulesToAbsolute();
    pitch = swerve.getPitch();
    swerve.drive(new Translation2d(-4, 0), /*may need to be changed*/
     0, true, true);
    
    
      while (!(pitch > stage1Threshold)){
        pitch = Math.abs(swerve.getPitch());
        SmartDashboard.putString("ChargeStation", "stage 1: " + pitch);
      }
      while (!(pitch < stage2Threshold)){
        pitch = Math.abs(swerve.getPitch());
        SmartDashboard.putString("ChargeStation", "stage 2: " + pitch);
      }
      finished = true;
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
