// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PhotonVision;
import frc.robot.subsystems.Swerve;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class FollowAprilTag extends CommandBase {
  private Swerve drivetrain;
  private PhotonVision camera;

  /** Creates a new FollowAprilTag. */
  public FollowAprilTag(Swerve dt, PhotonVision cam) {
    drivetrain = dt;
    camera = cam;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(dt, cam);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    camera.getNewResults();

    if (!camera.hasTarget()) {
      end(false);
    } else {
      double tx = camera.getTarget().getYaw();
      // The camera.getDistance() method returns the distance from the camera straight to the target
      // The math is to change it to the distance from the robot to the target on the floor
      double dist = Math.cos(Units.degreesToRadians(25))*camera.getDistance();
      Rotation2d rotation = new Rotation2d(tx);
      Translation2d translation = new Translation2d(dist-1, tx);
      drivetrain.drive(translation, 0, false, false);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}