// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.LEDLightControl;

public class SetColor extends CommandBase {
  private LEDLightControl leds;
  private Joystick gamepad;

  private Constants.ButtonHashtable bh = new Constants.ButtonHashtable();
  private Constants.ColorHashtable ch = new Constants.ColorHashtable();  

  private String lastPressedButton;
  /** Creates a new SetBlue. */
  public SetColor(LEDLightControl l, Joystick g) {
    // Use addRequirements() here to declare subsystem dependencies.
    leds = l;
    gamepad = g;
    addRequirements(leds);
  }

  public SetColor(LEDLightControl l) {
    leds = l;
    addRequirements(leds);
  }
  
  public void setColor (double val) {
    leds.colorSet(val);
  }
  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override//D - pad: up - yellow, left - purple, down - red, right - green | double press = off
  
  public void execute() {
    pressForColor("D_Pad_Up", "Yellow");
    pressForColor("D_Pad_Left", "Purple");
    pressForColor("D_Pad_Down", "Red");
    pressForColor("D_Pad_Right", "Green");
  }
  //press button --> leds go brrrrr
  private void pressForColor(String b, String c) {
    //If a button is pressed twice it turns it off
    if (lastPressedButton == b) {
      leds.colorSet(ch.colors.get("Black"));
      lastPressedButton = "";
    } else {
      lastPressedButton = b;
    }
    //If button b is pressed, set color to c
    if (gamepad.getRawButton(bh.buttons.get(b))) {
      leds.colorSet(ch.colors.get(c));
    }
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
