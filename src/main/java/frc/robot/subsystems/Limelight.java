// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {
  private boolean initialized = false;
  private NetworkTableEntry tTarget = null;
  private NetworkTableEntry tx = null;
  private NetworkTableEntry ty = null;
  private NetworkTableEntry ta = null;
  private NetworkTableEntry botpose = null;
  private NetworkTableEntry targetpose = null;

  /** Creates a new Limelight. */
  public Limelight() {
  }

  public void initializeLimeLight() {
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    turnOnLED();
    try {
      tTarget = table.getEntry("tv");
      tx = table.getEntry("tx");
      ty = table.getEntry("ty");
      ta = table.getEntry("ta");
      botpose = table.getEntry("botpose_wpiblue");
      targetpose = table.getEntry("targetpose_robotspace");
      
    } catch (Exception e) {
      SmartDashboard.putBoolean("couldn't get nt entries", true);
      return;
    }
    initialized = true;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    try {
      tTarget = table.getEntry("tv");
      tx = table.getEntry("tx");
      ty = table.getEntry("ty");
      ta = table.getEntry("ta");
      botpose = table.getEntry("botpose_wpiblue");
      targetpose = table.getEntry("targetpose_robotspace");
    } catch (Exception e) {
      return;
    }
    
  }

  public NetworkTableEntry getEntry(String str) {
    return NetworkTableInstance.getDefault().getTable("limelight").getEntry(str);
  }

  public boolean isInitialized() {
    return this.initialized;
  }

  public boolean hasTargets() {
    boolean hits = false;
    SmartDashboard.putBoolean("isInitialized", isInitialized());
    if (isInitialized()) {
      hits = (getEntry("tv").getDouble(0.0) == 1.0);
    }
    return hits;
  }

  public Double[] botPose() {
    Double[] botPose = null;
    if (isInitialized()) {
      botPose = botpose.getDoubleArray(new Double[7]); 
    }
    return botPose;
  }

  public double targetDist() {
    Double[] targetPose = null;
    if (isInitialized()) {
      targetPose = targetpose.getDoubleArray(new Double[3]);
    }
    Translation3d dist = new Translation3d(targetPose[0], targetPose[1], targetPose[2]);
    return dist.getDistance(new Translation3d());
  }

  public double x() {
    double dx = 0.0;
    if (isInitialized()) {
      dx = getEntry("tx").getDouble(0.0);
    }
    return dx;
  }

  public double y() {
    double dy = 0.0;
    if (isInitialized()) {
      dy = getEntry("ty").getDouble(0.0);
    }
    return dy;
  }

  public double targetArea() {
    double dArea = 0.0;
    if (isInitialized()) {
      dArea = getEntry("ta").getDouble(0.0);
    }
    return dArea;
  }

  public void turnOnLED() {
    lightLED(LimelightLED.ON);
  }

  public void turnOffLED() {
    lightLED(LimelightLED.OFF);
  }

  private void lightLED(LimelightLED value) {
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    table.getEntry("ledMode").setNumber(value.ordinal());
  }
}
