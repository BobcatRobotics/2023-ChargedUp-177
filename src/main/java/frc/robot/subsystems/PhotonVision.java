// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PhotonVision extends SubsystemBase {
  private PhotonCamera camera;
  private PhotonPipelineResult result;

  private final double CAMERA_HEIGHT = Units.inchesToMeters(5.25);
  private final double TARGET_HEIGHT = Units.inchesToMeters(42);
  private final double CAMERA_PITCH = Units.degreesToRadians(25);

  /** Creates a new PhotonVision. */
  public PhotonVision() {
    camera = new PhotonCamera("photonvision");
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

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}