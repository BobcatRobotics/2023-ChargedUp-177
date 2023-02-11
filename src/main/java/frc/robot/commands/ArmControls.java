// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;

public class ArmControls extends CommandBase {
  private Arm arm;
  private Joystick gamepad;

  public ArmControls(Arm a, Joystick gp) {
    arm = a;
    gamepad = gp;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(a);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Math.abs(gamepad.getRawAxis(1)) >= 0.05) { // Y axis on the left stick
      if (arm.isAtTopLimit() && gamepad.getRawAxis(1) > 0) {
        Constants.ElevatorConstants.canMove = false;
        arm.setSpeed(0);
      } else if (arm.isAtBottomLimit() && gamepad.getRawAxis(1) < 0) {
        Constants.ElevatorConstants.canMove = true;
        arm.setSpeed(0);
      } else {
        // Constants.ElevatorConstants.canMove = true;
        arm.setSpeed(gamepad.getRawAxis(1));
      }
    } else {
      // Constants.ElevatorConstants.canMove = true;
      arm.setSpeed(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
