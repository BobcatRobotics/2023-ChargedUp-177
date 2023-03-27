// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoAlign;

import java.util.List;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.PoseEstimator;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Wrist;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.DriveToPoseCommand;
import frc.robot.commands.Presets.Procedures.ScoreHigh;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html

public class TopMid extends SequentialCommandGroup {
  /** Creates a new TopMid. */
  Pose2d desiredpos;
  PoseEstimator sPose;
  Pose2d desiredPose;

  public TopMid(Swerve s, PoseEstimator sPose, Elevator e, Arm a, Wrist w, Intake i) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    this.sPose = sPose;
   
  
  
  

  
    
    addCommands(
      new DriveToPoseCommand(s, this::getGrid, sPose::getCurrentPose, true),
      new ScoreHigh(e, a, i, w)
    );
  }
  
  public Pose2d getGrid(){
    List<Pose2d> poses = List.of(
      Constants.PoseEstimation.grid1[1],
      Constants.PoseEstimation.grid2[1],
      Constants.PoseEstimation.grid3[1]
    );

    switch (poses.indexOf(sPose.getCurrentPose().nearest(poses))+1){
      case 1:
          return Constants.PoseEstimation.grid1[1];
      case 2:
          return Constants.PoseEstimation.grid2[1];
      case 3:
          return Constants.PoseEstimation.grid3[1];
      default:
          return Constants.PoseEstimation.grid2[1];
      }
  }
}

