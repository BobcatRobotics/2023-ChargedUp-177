// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Presets.Procedures;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Elevator;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.commands.Presets.RunIntake;
import frc.robot.commands.Presets.SetArm;
import frc.robot.commands.Presets.SetElevator;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;



public class ScoreHigh extends SequentialCommandGroup {
    // elevator high, arm out, wrist down, intake out
      public ScoreHigh(Elevator e, Arm a, Intake i) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(//will not work because we need to create the set elevator and arm commands
      new SequentialCommandGroup(
      Commands.parallel(new SetElevator(e),new SetArm(a)),
      new RunIntake(i,false,12))
    );
  }
}
