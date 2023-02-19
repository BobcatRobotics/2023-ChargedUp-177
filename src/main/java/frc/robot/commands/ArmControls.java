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
    // ArmConstants.armState = arm.getState();
    // SmartDashboard.putNumber("arm state", ArmConstants.armState);
    // // If none of the preset positions buttons are pressed, use continuous control
    // if (!gamepad.getRawButton(6) && !gamepad.getRawButton(7) && !gamepad.getRawButton(8)) {
    //   if (Math.abs(gamepad.getRawAxis(1)) >= 0.05) {
    //     if (arm.isAtTopLimit() && gamepad.getRawAxis(1) > 0) {
    //       arm.setSpeed(0);
    //     } else if (arm.isAtBottomLimit() && gamepad.getRawAxis(1) < 0) {
    //       arm.setSpeed(0);
    //     } else if (ElevatorConstants.elevatorState != 0 && arm.isAtConstrictedBottomLimit() && gamepad.getRawAxis(1) < 0) {
    //       arm.setSpeed(0);
    //     } else {
    //       arm.setSpeed(gamepad.getRawAxis(1));
    //     }
    //   } else {
    //     arm.setSpeed(0);
    //   }
    // } else {
    //   if (ElevatorConstants.elevatorState == 0) {
    //     if (gamepad.getRawButton(8)) { // right trigger
    //       arm.setState(2); // ground
    //     } else if (gamepad.getRawButton(7)) { // left trigger
    //       arm.setState(1); // middle
    //     } else if (gamepad.getRawButton(6)) { // right bumper
    //       arm.setState(0); // stowed
    //     }
    //   } else {
    //     if (gamepad.getRawButton(8)) { // right trigger
    //       arm.setState(2); // ground
    //     } else if (gamepad.getRawButton(7)) { // left trigger
    //       arm.setState(1); // middle
    //     }
    //   }
    // }
    if (Math.abs(gamepad.getRawAxis(1)) < 0.05) {
      arm.holdPosition();
    } else if (arm.isAtHardStop()) {
      arm.setSpeed(0);
    } else if (gamepad.getRawAxis(1) < 0) {
      arm.setSpeed(gamepad.getRawAxis(1)/7);
    } else {
      arm.setSpeed(gamepad.getRawAxis(1)/5);
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
