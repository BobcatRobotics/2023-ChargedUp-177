// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;//I tired

import frc.robot.Constants;
import frc.robot.Constants.elevator;
import frc.robot.subsystems.Elevator;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Joystick;

//YOO WHATS UP YOUTUBE

/** An example command that uses an example subsystem. */
public class Elevate extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Elevator elevator;
  private final boolean raise;
  
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public Elevate(Elevator subsystem, boolean alt) {
    elevator = subsystem;
    raise = alt;


    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // double current_position =elevator.get_encoder_distance(); 
    // int currState = elevator.get_state();
    //without comments it is

    double d = 0.0;
    // System.out.println(elevator.get_state());
    if (raise){
      d = 0.45;
    } else if (!raise){
      d =- 0.45;
    } 

    // if (d > 1.0){
    //   d=1.0;
    // }
    // else if (d < -1.0){
    //   d=-1.0;
    // }

    elevator.elevatorMotorset(d);
    
  }

  // Called once the command ends or is interrupted.
  
  @Override
  public void end(boolean interrupted) {
    elevator.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
     if((raise && elevator.getTopLimit()) || (!raise && elevator.getBottomLimit())){
       return true;
    }
    return false;
  }
}