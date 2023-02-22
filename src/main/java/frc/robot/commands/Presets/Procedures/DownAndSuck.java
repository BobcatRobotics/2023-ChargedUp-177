// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Presets.Procedures;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Presets.SetArm;
import frc.robot.commands.Presets.ZeroElevator;
import frc.robot.subsystems.Elevator;


public class DownAndSuck extends SequentialCommandGroup {
  // elevator down, arm up, wrist down, intake on until hard stop (voltage spike) is reached
  
  public DownAndSuck(Elevator e, Arm a) {

    addCommands(
      new ParallelCommandGroup(
        new ZeroElevator(e), //TODO: arm conflicts?
        new SetArm(),
      )
    );
  }
}
