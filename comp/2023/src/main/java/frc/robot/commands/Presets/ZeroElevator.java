// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Presets;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Elevator;

public class ZeroElevator extends CommandBase {
  /** Creates a new ZeroElevator. */
  Elevator e;
  public ZeroElevator(Elevator e) {
    this.e = e;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute(){
    while(e.getEncoderPos() > 20700){
      e.elevate(-0.75);
    }
    //bottom 10% of travel
    while(!e.getBottomLimits()){
      e.elevate(-0.5);
    }
    e.elevate(0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    e.resetEncoderPos();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return e.getBottomLimits();
  }
}
