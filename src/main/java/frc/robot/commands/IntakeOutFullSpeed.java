// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class IntakeOutFullSpeed extends CommandBase {
  /** Creates a new IntakeOut. */
  private Intake i;
  private Timer timer = new Timer();
  private boolean firstExecute = true;
  public IntakeOutFullSpeed(Intake i) {
    this.i = i;
    addRequirements(i);
  }

    // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.start();
  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(firstExecute) {
      firstExecute = false;
      timer.reset();
    }
    this.i.runIntakeOutFull();
  }
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(timer.hasElapsed(1.0)){
      firstExecute = true;
      this.i.stop();
      return true;
    }
    return false;
  } 
}
