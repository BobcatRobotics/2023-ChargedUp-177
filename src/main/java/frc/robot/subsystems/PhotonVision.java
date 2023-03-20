// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Optional;
import java.util.concurrent.atomic.AtomicReference;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import java.util.Optional;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class PhotonVision implements Runnable {
  private PhotonCamera camera;
  private PhotonPipelineResult result;
  private PhotonPoseEstimator photonPoseEstimator;
  private AtomicReference<EstimatedRobotPose> atomicEstimatedRobotPose = new AtomicReference<EstimatedRobotPose>();

  private final double CAMERA_HEIGHT = Units.inchesToMeters(6);
  private final double TARGET_HEIGHT = Units.inchesToMeters(16.5);
  private final double CAMERA_PITCH = Units.degreesToRadians(0);
  private final double APRILTAG_AMBIGUITY_THRESHOLD = 0.2;
  private final Translation3d ROBOT_TO_CAMERA_TRANSLATION = new Translation3d(0.5, 0.15, 0.05);
  private final Transform3d ROBOT_TO_CAMERA = new Transform3d(ROBOT_TO_CAMERA_TRANSLATION, new Rotation3d());
  private final double FIELD_LENGTH_METERS = 16.54175;
  private final double FIELD_WIDTH_METERS = 8.0137;



  /** Creates a new PhotonVision. */
  public PhotonVision() {
    camera = new PhotonCamera("photonvision");
    PhotonPoseEstimator photonPoseEstimator = null;
    try {
      var layout = AprilTagFields.k2023ChargedUp.loadAprilTagLayoutField();
      layout.setOrigin(OriginPosition.kBlueAllianceWallRightSide);
      if (camera != null) {
        photonPoseEstimator  = new PhotonPoseEstimator(layout, PoseStrategy.MULTI_TAG_PNP, camera, ROBOT_TO_CAMERA);
      }
    } catch (IOException e) {
      DriverStation.reportError("Failed to load AprilTagFieldLayout", e.getStackTrace());
      photonPoseEstimator = null;
    }
    this.photonPoseEstimator = photonPoseEstimator;
  }

  @Override
  public void run() {
    // Get AprilTag data
    if (photonPoseEstimator != null && camera != null && !RobotState.isAutonomous()) {
      var photonResults = camera.getLatestResult();
      if (photonResults.hasTargets() && (photonResults.targets.size() > 1 || photonResults.targets.get(0).getPoseAmbiguity() < APRILTAG_AMBIGUITY_THRESHOLD)) {
        photonPoseEstimator.update(photonResults).ifPresent(estimatedRobotPose -> {
          var estimatedPose = estimatedRobotPose.estimatedPose;
          if (estimatedPose.getX()  > 0.0 && estimatedPose.getX() <= FIELD_LENGTH_METERS && estimatedPose.getY() > 0.0 && estimatedPose.getY() <= FIELD_WIDTH_METERS) {
            atomicEstimatedRobotPose.set(estimatedRobotPose);
          }
        });
      }
    }
  }

  public EstimatedRobotPose grabLatestEstimatedPose() {
    return atomicEstimatedRobotPose.getAndSet(null);
  }
}