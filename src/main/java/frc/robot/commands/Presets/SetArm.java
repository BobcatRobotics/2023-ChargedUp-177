// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Presets;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;

public class SetArm extends CommandBase {
  /** Creates a new SetArm. */
  Arm arm;
  int state;
  Timer timer;
  int pos;
  public SetArm(Arm a, int state_g) {
    state = state_g;
    arm = a;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(a);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (arm.isAtStowedLimit() && state == 0) {
      arm.setSpeed(0);
    }
    
    if (state == 0){
      arm.setState(0);
    }
    else if(state == 1){
      arm.setState(1);
    }
    else if(state == 2){
      arm.setState(2);
    } else if (state == 3) {
      arm.setState(3);
    } else if(state == 4){
      arm.setState(4);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putBoolean("arm interrupted", interrupted);
    if(arm.isAtBottomLimit()){
      arm.resetEncoder();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (state == 0) {
      pos = Constants.ArmConstants.pos0;
    } else if (state == 1) {
      pos = Constants.ArmConstants.pos1;
    } else if (state == 2) {
      pos = Constants.ArmConstants.pos2;
    } else if (state == 3) {
      pos = Constants.ArmConstants.bottomPickup;
    }
    // if(arm.getState() == state){
    //   return true;
    // }
    // return false;
    if (arm.getPos() >= pos-200 && arm.getPos() <= pos) {

      // SmartDashboard.putBoolean("elevator timer finished", timer.hasElapsed(5));

      return true;
    }
    return false;
  }
}
