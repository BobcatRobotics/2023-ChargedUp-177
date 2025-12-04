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

  // TODO: Tune!
  private final double P_GAIN = 0.7;
  private final double D_GAIN = 0.0;
  private PIDController controller = new PIDController(P_GAIN, 0, D_GAIN);

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
    double forwardSpeed = 0;
    camera.getNewResults();

    if (!camera.hasTarget()) {
      end(false);
    } else {
      double yaw = camera.getTarget().getYaw();
      double range = camera.getDistance();

      double rangeX = range * Math.cos(Units.degreesToRadians(yaw));
      double rangeY = range * Math.sin(Units.degreesToRadians(yaw));

      double forwardSpeedX = -controller.calculate(rangeX, 1);
      //double forwardSpeedY = -controller.calculate(rangeY, 0.5);
      Translation2d translation = new Translation2d(forwardSpeedX, 0);

      drivetrain.drive(translation, 0, false, false);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.drive(new Translation2d(), 0, false, true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}