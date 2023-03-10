// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Presets.Procedures;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.Presets.RunIntake;
import frc.robot.commands.Presets.SetArm;
import frc.robot.commands.Presets.SetElevator;
import frc.robot.commands.Presets.SetWrist;
import frc.robot.commands.Presets.ZeroElevator;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Wrist;


public class ForwardSuck extends SequentialCommandGroup {
  // elevator down, arm up, wrist down
  
  public ForwardSuck(Elevator e, Arm a, Intake i, Wrist w) {
    if(e.getEncoderPos() >= Constants.ElevatorConstants.pos1){
      addCommands(
        new ParallelCommandGroup(
          new InstantCommand(() -> a.setPos(Constants.ArmConstants.minNonCollidingExtention)),
          new SetWrist(w, true)
        ),
        new SetElevator(e, 0)
      );
    }else{
    addCommands(
      Commands.parallel(new SetElevator(e,0),new SetArm(a,3), new SetWrist(w,true))
    );
    }
  }
}
