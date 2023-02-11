// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/*
1. Figure out angleOffset from target
2. Send drive command with that rotation
*/

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.PhotonVision;
import frc.robot.subsystems.Swerve;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class AlignToAprilTag extends CommandBase {
  private Swerve drivetrain;
  private PhotonVision camera;
  
  // TODO: Tune!
  private final double ANGULAR_P = -0.05;
  private final double ANGULAR_D = 0.0;
  private PIDController turnController = new PIDController(ANGULAR_P, 0, ANGULAR_D);

  /** Creates a new AlignToAprilTag. */
  public AlignToAprilTag(Swerve dt, PhotonVision cam) {
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
    double rotationSpeed = 0;
    camera.getNewResults();

    if (!camera.hasTarget()) {
      end(false);
    } else {
      double yaw = camera.getTarget().getYaw();
      rotationSpeed = -turnController.calculate(yaw, 0);
      drivetrain.drive(new Translation2d(0, 0), rotationSpeed, false, false);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.drive(new Translation2d(), 0, true, true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}