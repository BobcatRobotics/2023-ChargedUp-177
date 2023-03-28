// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.LEDs;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.BlinkinLEDs;

public class BlinkPurple extends CommandBase {
  /** Creates a new BlinkYellow. */
  Timer t;
  BlinkinLEDs leds;
  public BlinkPurple(BlinkinLEDs leds) {
    // Use addRequirements() here to declare subsystem dependencies.
    t = new Timer();
    this.leds = leds;
    addRequirements(leds);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    leds.setBlack();
    t.reset();
    t.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(!t.hasElapsed(1)){
      leds.blinkPurple();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    leds.setPurple();
    t.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return t.hasElapsed(1);
  }
}
