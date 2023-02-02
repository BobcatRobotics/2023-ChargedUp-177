// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Swerve;

public class AlignToTarget extends CommandBase {
  private Swerve drivetrain;
  private Limelight lime;
  private PIDController pidController;

  private double kp = 0.5;
  private double ki = 0;
  private double kd = 0;
  private double setpoint = 0;
  private double tolerance = 0.5;

  private double xOffset;

  /** Creates a new AlignToTarget. */
  public AlignToTarget(Swerve dt, Limelight lm) {
    drivetrain = dt;
    lime = lm;

    pidController = new PIDController(kp, ki, kd);
    pidController.setSetpoint(setpoint);
    pidController.setTolerance(tolerance);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(dt);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    lime.turnOnLED();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (lime.hasTargets() && !isFinished()) {
      xOffset = lime.x();
      drivetrain.drive(new Translation2d(), pidController.calculate(xOffset), true, true);
    } else {
      end(false);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    lime.turnOffLED();
    drivetrain.drive(new Translation2d(), 0, true, true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pidController.atSetpoint();
  }
}
