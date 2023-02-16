package frc.robot.autos;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class exampleAuto extends SequentialCommandGroup {
    String trajectoryJSON = "paths/test11823.wpilib.json";
    //String trajectoryJSON2 = "paths/test11923.wpilib.json";
    Trajectory exampleTrajectory = new Trajectory();
    //Trajectory exampleTrajectory2 = new Trajectory();

    public exampleAuto(Swerve s_Swerve){
        TrajectoryConfig config =
            new TrajectoryConfig(
                    Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                    Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                .setKinematics(Constants.Swerve.swerveKinematics);

        // // An example trajectory to follow.  All units in meters.
        // Trajectory exampleTrajectory =
        //     TrajectoryGenerator.generateTrajectory(
        //         // Start at the origin facing the +X direction
        //         new Pose2d(0, 0, new Rotation2d(0)),
        //         // Pass through these two interior waypoints, making an 's' curve path
        //         List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
        //         // End 3 meters straight ahead of where we started, facing forward
        //         new Pose2d(3, 0, new Rotation2d(0)),
        //         config);

        try {
            Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
            //Path trajectoryPath2 = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON2);
            exampleTrajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
            //exampleTrajectory2 = TrajectoryUtil.fromPathweaverJson(trajectoryPath2);
        } catch (IOException ex) {
            DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
        }

        var thetaController =
            new ProfiledPIDController(
                Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        SwerveControllerCommand swerveControllerCommand =
            new SwerveControllerCommand(
                exampleTrajectory,
                s_Swerve::getPose,
                Constants.Swerve.swerveKinematics,
                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                thetaController,
                s_Swerve::setModuleStates,
                s_Swerve);

        // SwerveControllerCommand swerveControllerCommand2 =
        //     new SwerveControllerCommand(
        //         exampleTrajectory2,
        //         s_Swerve::getPose,
        //         Constants.Swerve.swerveKinematics,
        //         new PIDController(Constants.AutoConstants.kPXController, 0, 0),
        //         new PIDController(Constants.AutoConstants.kPYController, 0, 0),
        //         thetaController,
        //         s_Swerve::setModuleStates,
        //         s_Swerve);                

        // TODO: change angle and position odometry of next path for follow and align to lime/april
        // new InstantCommand(() -> s_Swerve.resetOdometry(exampleTrajectory2.getInitialPose())), swerveControllerCommand2, 
        addCommands(
            new InstantCommand(() -> s_Swerve.resetOdometry(exampleTrajectory.getInitialPose())),
            swerveControllerCommand, new WaitCommand(2)
        );
    }
}