// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Presets.Procedures;
import frc.robot.commands.Presets.RunIntake;
import frc.robot.commands.Presets.SetArm;
import frc.robot.commands.Presets.SetElevator;
import frc.robot.commands.Presets.SetWrist;
import frc.robot.commands.Presets.SetElevator;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.WristState;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;


public class TopSuck extends SequentialCommandGroup {
  // elevator down, arm down, wrist up, intake untill hard stop

  public TopSuck(Elevator e, Arm a, Intake i, Wrist W) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    //always set the wrist to false for everything 
    addCommands(
      Commands.parallel(new SetElevator(e,1),new SetArm(a,2), new SetWrist(W, WristState.topGround))
    );
  }
}
