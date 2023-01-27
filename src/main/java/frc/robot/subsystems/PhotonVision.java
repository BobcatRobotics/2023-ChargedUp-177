// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.Optional;

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
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class PhotonVision extends SubsystemBase {
  private PhotonCamera camera;
  private PhotonPipelineResult result;
  private PhotonPoseEstimator photonPoseEstimator;

  private final double CAMERA_HEIGHT = Units.inchesToMeters(6);
  private final double TARGET_HEIGHT = Units.inchesToMeters(16.5);
  private final double CAMERA_PITCH = Units.degreesToRadians(0);
  private final Translation3d ROBOT_TO_CAMERA_TRANSLATION = new Translation3d(0.5, 0.15, 0.05);
  private final Transform3d ROBOT_TO_CAMERA = new Transform3d(ROBOT_TO_CAMERA_TRANSLATION, new Rotation3d());



  /** Creates a new PhotonVision. */
  public PhotonVision() {
    camera = new PhotonCamera("photonvision");

    final AprilTag tag6 = new AprilTag(6, new Pose3d(Units.inchesToMeters(45), 0, TARGET_HEIGHT, null));
    ArrayList<AprilTag> atList = new ArrayList<AprilTag>();
    atList.add(tag6);

    // TODO - once 2023 happens, replace this with just loading the 2023 field arrangement
    AprilTagFieldLayout atfl = new AprilTagFieldLayout(atList, Constants.FieldConstants.length, Constants.FieldConstants.width);
    photonPoseEstimator = new PhotonPoseEstimator(atfl, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, camera, ROBOT_TO_CAMERA);
  }

  public void getNewResults() {
    result = camera.getLatestResult();
  }
  
  public boolean hasTarget() {
    return result.hasTargets();
  }

  public PhotonTrackedTarget getTarget() {
    return result.getBestTarget();
  }

  // returns distance to target in meters
  public double getDistance() {
    return PhotonUtils.calculateDistanceToTargetMeters(CAMERA_HEIGHT, TARGET_HEIGHT, CAMERA_PITCH, Units.degreesToRadians(result.getBestTarget().getPitch()));
  }

  public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
    photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
    return photonPoseEstimator.update();  
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}