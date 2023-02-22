// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Presets;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj.Joystick;


public class RunIntake extends CommandBase {
  /** Creates a new RunIntake. */
  Intake i;
  boolean runIn;
  double time;
  Joystick gp;
  int in = 5;
  int out = 6;
  public RunIntake(Intake i, boolean runIn, double time){
    this.i = i;
    this.runIn = runIn;
    this.time = time;
    addRequirements(i);
  }
  public RunIntake(Intake i, Joystick gp){
    this.i = i;
    this.in = in;
    this.gp = gp;
    addRequirements(i);
  }
 
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   if(gp.getRawButton(in)){
      i.runIntakeIn();
    } else if(gp.getRawButton(out)){
      i.runIntakeOut();
    }else{
      i.stop();
    }
    }
  

  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    i.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(gp.getRawButton(in)){
      return false;
    }else if (gp.getRawButton(out)){
      return false;
    }else{
      return true;
    }
  }
}

