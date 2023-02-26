// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Presets;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator; 

public class SetElevator extends CommandBase {
  Elevator elevator;
  int state;
  Timer timer;
  /** Creates a new SetElevator. */
  public SetElevator(Elevator e, int state_g) {
    elevator = e;
    state = state_g;
    timer = new Timer();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(e);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if(state == 0){
      elevator.setState(0);
    }
    else if(state == 1){
      elevator.setState(1);
    }
    else if(state == 2){
      elevator.setState(2);
    }


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putBoolean("elevator interrupted", interrupted);
    if (elevator.getBottomLimits() && state == 0){
      elevator.resetEncoderPos();
    }
    elevator.holdPosition();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // if(elevator.getState() == state){
    //   return true;
    // }
    // return false;
    if (timer.hasElapsed(1)) {
      timer.stop();
      timer.reset();
      return true;
    }
    return false;
  }
}
