// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.Intake;

/** An example command that uses an example subsystem. */
public class Intakegoburrrr extends CommandBase 
{
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Intake intake;
  private boolean rightPressed;
  private boolean leftPressed;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public Intakegoburrrr(Intake intake_init, boolean rightPressed_init, boolean leftPressed_init) 
  {
    intake = intake_init;
    rightPressed = rightPressed_init;
    leftPressed = leftPressed_init;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake);
    
  }
  

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    
    if (rightPressed)
    {
      intake.intakeForward();
    }
    else if (leftPressed) 
    {
      intake.reverseIntake();
    }


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    intake.intakeStop();
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() 
  {
    if ( (rightPressed == false && leftPressed == false) || (rightPressed == false && leftPressed != true) || (leftPressed == false && rightPressed != true))
    {
      return true;
    }
    return false;
  }
}
