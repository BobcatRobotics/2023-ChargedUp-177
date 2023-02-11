// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.Elevator;

public class ElevatorControls extends CommandBase {
  private Elevator elevator;
  private Joystick gamepad;

  /** Creates a new ElevatorControls. */
  public ElevatorControls(Elevator e, Joystick gp) {
    elevator = e;
    gamepad = gp;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ElevatorConstants.elevatorState = elevator.getState();
    SmartDashboard.putNumber("elevator state", ElevatorConstants.elevatorState);
    // If none of the preset positions buttons are pressed, use continuous control
    if (gamepad.getPOV() == -1) {
      if (Math.abs(gamepad.getRawAxis(3)) >= 0.05 && ArmConstants.armState != 0) {
        if (elevator.isAtTopLimit() && gamepad.getRawAxis(3) > 0) {
          elevator.elevate(0);
        } else if (elevator.isAtBottomLimit() && gamepad.getRawAxis(3) < 0) {
          elevator.elevate(0);
        } else {
          elevator.elevate(gamepad.getRawAxis(3));
        }
      } else {
        elevator.elevate(0);
      }
    } else {
      if (ArmConstants.armState != 0) {
        if (gamepad.getPOV() == 0) {
          elevator.setState(2);
        } else if (gamepad.getPOV() == 90) {
          elevator.setState(1);
        } else if (gamepad.getPOV() == 180) {
          elevator.setState(0);
        }
      }
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
