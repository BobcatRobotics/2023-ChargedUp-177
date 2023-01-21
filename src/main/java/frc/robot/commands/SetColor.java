// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LEDLightControl;

public class SetColor extends CommandBase {
  private LEDLightControl leds;
  private Joystick gamepad;

  private double colorDub = 0.57;

  
  /** Creates a new SetBlue. */
  public SetColor(LEDLightControl l, Joystick g) {
    // Use addRequirements() here to declare subsystem dependencies.
    leds = l;
    gamepad = g;
    addRequirements(leds);
  }

  public SetColor(LEDLightControl l) {
    leds = l;
    addRequirements(leds);
  }
  
  public void setColor (double val) {
    leds.colorSet(val);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   
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
