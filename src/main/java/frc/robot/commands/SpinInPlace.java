// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;

public class SpinInPlace extends CommandBase {
  /** Creates a new SpinInPlace. */
  private Swerve swerve;
  private boolean firstRun;
  private Timer timer;

  public SpinInPlace(Swerve s) {
    // Use addRequirements() here to declare subsystem dependencies.
    swerve = s;
    firstRun = true;
    timer = new Timer();
    addRequirements(s);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    firstRun = true;
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (firstRun) {
      swerve.drive(new Translation2d(), 2*Math.PI, false, false);
      firstRun = false;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerve.drive(new Translation2d(), 0, false, true);
    timer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !firstRun && timer.hasElapsed(0.5);
  }
}
