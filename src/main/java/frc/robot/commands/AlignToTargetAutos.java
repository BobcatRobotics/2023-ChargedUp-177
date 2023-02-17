// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Swerve;

public class AlignToTargetAutos extends CommandBase {
  private Swerve drivetrain;
  private Limelight lime;
  private PIDController pidController;
  private Timer timer;

  private double kp = 0.2;
  private double ki = 0;
  private double kd = 0;
  private double setpoint = 0;
  private double tolerance = 0.1;

  private double xOffset;
  private double calc;

  /** Creates a new AlignToTarget. */
  public AlignToTargetAutos(Swerve dt, Limelight lm) {
    drivetrain = dt;
    lime = lm;
    timer = new Timer();

    pidController = new PIDController(kp, ki, kd);
    pidController.setSetpoint(setpoint);
    pidController.setTolerance(tolerance);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(dt);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.start();
    lime.turnOnLED();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putBoolean("Executing", true);
    SmartDashboard.putBoolean("HasTarget", lime.hasTargets());
    SmartDashboard.putBoolean("has targets",lime.hasTargets());
    SmartDashboard.putBoolean("align finished", isFinished());
    if (lime.hasTargets() & !isFinished()) {
      xOffset = lime.x();
      calc = pidController.calculate(xOffset);
      SmartDashboard.putNumber("xOffset", xOffset);
      SmartDashboard.putNumber("PID Value", calc);
      drivetrain.drive(new Translation2d(), calc, true, true);
    } else {
      end(false);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.drive(new Translation2d(), 0, true, true);
    SmartDashboard.putBoolean("Executing", false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.hasElapsed(2);
    //return pidController.atSetpoint();
  }
}
