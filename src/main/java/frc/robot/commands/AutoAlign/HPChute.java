// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoAlign;

import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.PoseEstimator;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Wrist;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.DriveToPoseCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class HPChute extends SequentialCommandGroup {
  /** Creates a new HPChute. */
  public HPChute(Swerve s, PoseEstimator sPose, Elevator e, Arm a, Wrist w, Intake i) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new DriveToPoseCommand(s, () -> Constants.PoseEstimation.hpStation, sPose::getCurrentPose, true)
      //TODO: add intake preset
    );
  }
}
