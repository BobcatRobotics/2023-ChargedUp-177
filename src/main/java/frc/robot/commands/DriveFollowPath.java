// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/* Follows a path planner path */

package frc.robot.commands;

import javax.print.attribute.standard.Destination;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class DriveFollowPath extends CommandBase {
    Timer timer = new Timer();
    PathPlannerTrajectory trajectory;
    HolonomicDriveController controller;
    boolean resetOdometry;

    String pathName;

    public DriveFollowPath(String pathname) {
        this(pathname, Constants.AutoConstants.kMaxSpeedMetersPerSecond, Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared, true);
    }

    public DriveFollowPath(String pathname, double maxVel, double maxAccel) {
        this(pathname, maxVel, maxAccel, true);
    }

    public DriveFollowPath(String pathName, double maxVel, double maxAccel, boolean resetOdometry) {
        addRequirements(RobotContainer.s_Swerve);

        this.trajectory = PathPlanner.loadPath(pathName, maxVel, maxAccel);

        PIDController xController = new PIDController(Constants.AutoConstants.kPXController, 0, 0);
        PIDController yController = new PIDController(Constants.AutoConstants.kPYController, 0, 0);
        ProfiledPIDController thetaController = new ProfiledPIDController(
                Constants.AutoConstants.kPThetaController, 0, 0,
                new TrapezoidProfile.Constraints(Constants.AutoConstants.kMaxAngularSpeedRadiansPerSecond,
                        Constants.AutoConstants.kMaxAngularSpeedRadiansPerSecondSquared));
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        this.controller = new HolonomicDriveController(xController, yController, thetaController);
        this.resetOdometry = resetOdometry;

        this.pathName = pathName;
    }

    public DriveFollowPath(PathPlannerTrajectory traj, boolean resetOdometry) {
        addRequirements(RobotContainer.s_Swerve);
        
        this.trajectory = traj;

        PIDController xController = new PIDController(Constants.AutoConstants.kPXController, 0, 0);
        PIDController yController = new PIDController(Constants.AutoConstants.kPYController, 0, 0);
        ProfiledPIDController thetaController = new ProfiledPIDController(
                Constants.AutoConstants.kPThetaController, 0, 0,
                new TrapezoidProfile.Constraints(Constants.AutoConstants.kMaxAngularSpeedRadiansPerSecond,
                        Constants.AutoConstants.kMaxAngularSpeedRadiansPerSecondSquared));
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        this.controller = new HolonomicDriveController(xController, yController, thetaController);
        this.resetOdometry = resetOdometry;        
    }

    public PathPlannerTrajectory getTraj() {
        return trajectory;
    }

    @Override
    public void initialize() {
        RobotContainer.s_Swerve.enableBrakeMode(true);
        timer.reset();
        timer.start();
        Pose2d initialPose = trajectory.getInitialHolonomicPose();
        SmartDashboard.putString("initialPose", initialPose.toString());
        if(resetOdometry) RobotContainer.s_Swerve.resetOdometry(new Pose2d(initialPose.getTranslation(), initialPose.getRotation())); // RobotContainer.s_Swerve.getYaw()
    }

    @Override
    public void execute(){
        double time = timer.get();
        PathPlannerState desiredState = (PathPlannerState) trajectory.sample(time);
        ChassisSpeeds targetSpeeds = controller.calculate(RobotContainer.s_Swerve.getPose(), desiredState, new Rotation2d(desiredState.holonomicRotation.getRadians()));

        targetSpeeds.vyMetersPerSecond = targetSpeeds.vyMetersPerSecond;
        targetSpeeds.omegaRadiansPerSecond = targetSpeeds.omegaRadiansPerSecond;

        // System.out.println("x:   " + RobotContainer.drive.getPoseMeters().getTranslation().getX() + " y:   " + RobotContainer.drive.getPoseMeters().getTranslation().getY() + " r: " + RobotContainer.drive.getPoseMeters().getRotation().getDegrees());
        // System.out.println("tx:  " + desiredState.poseMeters.getTranslation().getX() + " ty: " + desiredState.poseMeters.getTranslation().getY() + " tr:" + desiredState.holonomicRotation.getDegrees());
        // System.out.println("tvx: " + targetSpeeds.vxMetersPerSecond + " tvy: " + targetSpeeds.vyMetersPerSecond);
        // Position PID
        // SmartDashboard.putNumber("PIDTarget", 0);
        // SmartDashboard.putNumber("PIDActual", pathController.getPosError());

        // Rotation PID
        // SmartDashboard.putNumber("PIDTarget", desiredState.holonomicRotation.getDegrees());
        // SmartDashboard.putNumber("PIDACtual", RobotContainer.drive.getAngleDegrees());

        // Heading PID
        // SmartDashboard.putNumber("PIDTarget", desiredState.poseMeters.getRotation().getDegrees());
        // SmartDashboard.putNumber("PIDActual", pathController.getCurrentHeading().getDegrees());
        // System.out.println("tr:" + Math.round(desiredState.holonomicRotation.getDegrees()) + ", " + "r:" + Math.round(RobotContainer.drive.getAngleDegrees()) + " | th:" + Math.round(desiredState.poseMeters.getRotation().getDegrees()));

        Pose2d currentPose = RobotContainer.s_Swerve.getPose();
        String tString = " [" + Math.round(timer.get() * 100) / 100.0 + "]";
        System.out.println(pathName + tString + " x error: " + (desiredState.poseMeters.getX() - currentPose.getX()));
        System.out.println(pathName + tString + " y error: " + (desiredState.poseMeters.getY() - currentPose.getY()));
        System.out.println(pathName + tString + " r error: " + (desiredState.holonomicRotation.getDegrees() - currentPose.getRotation().getDegrees()));

        RobotContainer.s_Swerve.drive(targetSpeeds);
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();
        RobotContainer.s_Swerve.drive(new Translation2d(0, 0), 0.0, false, true);
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(trajectory.getTotalTimeSeconds());
    }
}