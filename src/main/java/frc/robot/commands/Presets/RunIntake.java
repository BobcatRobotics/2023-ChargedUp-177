// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Presets;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;


public class RunIntake extends CommandBase {
  /** Creates a new RunIntake. */
  Intake i;
  boolean runIn;
  double time;
  boolean isTimed = false;
  Joystick gp;
  Timer timer;
  int in = 5;
  int out = 7;
  int back = 9;
  public RunIntake(Intake i, boolean runIn, double time){
    this.i = i;
    this.runIn = runIn;
    this.time = time;
    timer = new Timer();
    isTimed = true;
    addRequirements(i);
  }
  public RunIntake(Intake i, Joystick gp){
    this.i = i;
    this.in = in;
    this.gp = gp;
    timer = new Timer();
    addRequirements(i);
  }
 
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(!isTimed) {
      if(gp.getRawButton(in)){
        i.runIntakeIn();
      } else if(gp.getRawButton(out)){
        i.runIntakeOut();
      }else if(gp.getRawButton(back)) {
        i.runIntakeOutFull();
      } else {
        i.stop();
      }
    }else{
      timer.start();
      if(!timer.hasElapsed(time)){
        if(runIn){
          i.runIntakeIn();
        }else{
          i.runIntakeOut();
        }
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    i.stop();
    isTimed = false;
    timer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // if(gp.getRawButton(in)){
    //   return false;
    // }else if (gp.getRawButton(out)){
    //   return false;
    // }else if(gp.getRawButton(back)){
    //   return false;
    // }else{
    //   //return true;
    //   return false;
    // }
    if (!isTimed) {
      return false;
    } else {
      return timer.hasElapsed(time);
    }
  }
}

