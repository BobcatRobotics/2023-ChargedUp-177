// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Presets;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class SetArm extends CommandBase {
  /** Creates a new SetArm. */
  Arm arm;
  int state;
  Timer timer;
  public SetArm(Arm a, int state_g) {
    state = state_g;
    arm = a;
    timer = new Timer();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(a);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
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
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putBoolean("arm interrupted", interrupted);
    if(arm.isAtBottomLimit()){
      arm.resetEncoder();
    }
    arm.holdPosition();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // if(arm.getState() == state){
    //   return true;
    // }
    // return false;
    if (timer.hasElapsed(2)) {
      timer.stop();
      timer.reset();
      return true;
    }
    return false;
  }
}
