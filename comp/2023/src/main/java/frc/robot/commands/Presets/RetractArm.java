// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Presets;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Arm;

public class RetractArm extends CommandBase {
  //TODO: Write this command
  
  // if the elevator is at the bottom, retract the arm
  // otherwise, do nothing
  Arm a;
  Elevator e ;


  
   public RetractArm(Elevator e, Arm a) {
   this.e = e;
   this.a = a; 
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(e.getState() == 0 && a.getState() != 0){
      a.setState(0);
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    a.resetEncoder();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return a.isAtBottomLimit();
  }
}
