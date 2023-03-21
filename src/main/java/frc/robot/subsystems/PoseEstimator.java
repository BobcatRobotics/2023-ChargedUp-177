// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Supplier;

import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

public class PoseEstimator {
  private static final Vector<N3> stateStdDevs = VecBuilder.fill(0.01, 0.01, 1.5);
  private static final Vector<N3> visionMeasurementStdDevs = VecBuilder.fill(1.5, 1.5, 1.5);

  private final Supplier<Rotation2d> rotationSupplier;
  private final Supplier<SwerveModulePosition[]> modulePositionSupplier;
  private final SwerveDrivePoseEstimator poseEstimator;
  //private PhotonVision photon = new PhotonVision();
  private Limelight limelight;

  private OriginPosition originPosition = OriginPosition.kBlueAllianceWallRightSide;
  private boolean sawTag = false;

  private final double FIELD_LENGTH_METERS = 16.54175;
  private final double FIELD_WIDTH_METERS = 8.0137;
  
  /** Creates a new PoseEstimatorSubsystem. */
  public PoseEstimator(Supplier<Rotation2d> rotationSupplier, Supplier<SwerveModulePosition[]> modulePositionSupplier, Limelight limelight) {
    this.rotationSupplier = rotationSupplier;
    this.modulePositionSupplier = modulePositionSupplier;

    poseEstimator = new SwerveDrivePoseEstimator(
      Constants.Swerve.swerveKinematics, 
      this.rotationSupplier.get(), 
      this.modulePositionSupplier.get(), 
      new Pose2d(),
      stateStdDevs,
      visionMeasurementStdDevs
    );
    this.limelight = limelight;
  }

  public void setAlliance(Alliance alliance) {
    boolean allianceChanged = false;
    switch(alliance) {
      case Blue:
        allianceChanged = (originPosition == OriginPosition.kRedAllianceWallRightSide);
        originPosition = OriginPosition.kBlueAllianceWallRightSide;
        break;
      case Red:
        allianceChanged = (originPosition == OriginPosition.kBlueAllianceWallRightSide);
        originPosition = OriginPosition.kRedAllianceWallRightSide;
        break;
      default:
    }

    if (allianceChanged && sawTag) {
      Pose2d newPose = flipAlliance(getCurrentPose());
      poseEstimator.resetPosition(
        this.rotationSupplier.get(), 
        this.modulePositionSupplier.get(), 
        newPose
      );
    }
  }

  public void periodic() {
    // This method will be called once per scheduler run
    poseEstimator.update(this.rotationSupplier.get(), this.modulePositionSupplier.get());
    // var visionPose = photon.grabLatestEstimatedPose();
    // if (visionPose != null) {
    //   sawTag = true;
    //   Pose2d pose2d = visionPose.estimatedPose.toPose2d();
    //   if (originPosition != OriginPosition.kBlueAllianceWallRightSide) {
    //     pose2d = flipAlliance(pose2d);
    //   }
    //   poseEstimator.addVisionMeasurement(pose2d, visionPose.timestampSeconds);
    // }
    Pose3d visionPose = null;
    try {
      visionPose = new Pose3d(limelight.botPose()[0], limelight.botPose()[1], limelight.botPose()[2], new Rotation3d(limelight.botPose()[3], limelight.botPose()[4], limelight.botPose()[5]));
    } catch (Exception e) {}

    if (visionPose != null) {
      sawTag = true;
      Pose2d pose2d = visionPose.toPose2d();
      if (originPosition != OriginPosition.kBlueAllianceWallRightSide) {
        pose2d = flipAlliance(pose2d);
      }
      double distance = limelight.targetDist();
      poseEstimator.addVisionMeasurement(pose2d, Timer.getFPGATimestamp() - limelight.botPose()[6]/1000.0, VecBuilder.fill(distance/2, distance/2, 0));
    }
  }

  private String getFormattedPose() {
    Pose2d pose = getCurrentPose();
    return String.format("(%.3f, %.3f) %.2f degrees",
      pose.getX(),
      pose.getY(),
      pose.getRotation().getDegrees()
    );
  }

  public Pose2d getCurrentPose() {
    return poseEstimator.getEstimatedPosition();
  }

  public void setCurrentPose(Pose2d newPose) {
    poseEstimator.resetPosition(this.rotationSupplier.get(), this.modulePositionSupplier.get(), newPose);
  }

  public void resetFieldPosition() {
    setCurrentPose(new Pose2d());
  }

  private Pose2d flipAlliance(Pose2d poseToFlip) { // field length + field width + 180 degree rotation
    return poseToFlip.relativeTo(new Pose2d(new Translation2d(16.54175, 8.0137), new Rotation2d(Math.PI)));
  }
}