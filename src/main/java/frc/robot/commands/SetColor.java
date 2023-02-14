// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.LEDLightControl;

public class SetColor extends CommandBase {
  private LEDLightControl leds = new LEDLightControl();

  private Constants.ButtonHashtable bh = new Constants.ButtonHashtable();
  private Constants.ColorHashtable ch = new Constants.ColorHashtable();  

  private String lastPressedButton;

  //the new LEDLightControl should jus tbe instantiated here due to there being no arguments, but it was already used
  //in BalanceChargeStation
  public SetColor(String c) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(leds);
    setColor(c);
  }
  //when the setColor needs to be used directly, and not when the command itself is called
  public SetColor() {
    addRequirements(leds);
    //when the method needs to be called directly, set it to black first
    setColor("Black");
  }
  
  
  public void setColor (String co) {
    leds.colorSet(ch.colors.get(co));
  }
  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override//D - pad: up - yellow, left - purple, down - red, right - green | double press = off
  
  public void execute() {
  }
  
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
