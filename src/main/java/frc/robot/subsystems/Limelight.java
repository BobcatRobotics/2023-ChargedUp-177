// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.Vector;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {
  private NetworkTable table;

  private boolean initialized = false;
  private NetworkTableEntry tTarget = null;
  private NetworkTableEntry tx = null;
  private NetworkTableEntry ty = null;
  private NetworkTableEntry ta = null;
  private NetworkTableEntry tl = null;
  private NetworkTableEntry cl = null;
  private NetworkTableEntry botpose = null;
  private NetworkTableEntry campose = null;

  /** Creates a new Limelight. */
  public Limelight() {
  }

  public void initializeLimeLight() {
    table = NetworkTableInstance.getDefault().getTable("limelight");
    turnOnLED();
    try {
      tTarget = table.getEntry("tv");
      tx = table.getEntry("tx");
      ty = table.getEntry("ty");
      ta = table.getEntry("ta");
      tl = table.getEntry("tl");
      cl = table.getEntry("cl");
      botpose = table.getEntry("botpose_wpiblue");
      campose = table.getEntry("camerapose_robotspace");
      
    } catch (Exception e) {
      SmartDashboard.putBoolean("couldn't get nt entries", true);
      return;
    }
    initialized = true;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    table = NetworkTableInstance.getDefault().getTable("limelight");
    try {
      tTarget = table.getEntry("tv");
      tx = table.getEntry("tx");
      ty = table.getEntry("ty");
      ta = table.getEntry("ta");
      tl = table.getEntry("tl");
      cl = table.getEntry("cl");
      botpose = table.getEntry("botpose_wpiblue");
      campose = table.getEntry("camerapose_robotspace");
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
      botPose = botpose.getDoubleArray(new Double[6]);
    }
    return botPose;
  }

  public Double[] camPose() {
    Double[] camPose = null;
    if (isInitialized()) {
      camPose = campose.getDoubleArray(new Double[6]);
    }
    return camPose;
  }

  public double tl() {
    double tl = 0.0;
    if (isInitialized()) {
      tl = getEntry("tl").getDouble(0.0);
    }
    return tl;
  }

  public double cl() {
    double cl = 0.0;
    if (isInitialized()) {
      cl = getEntry("cl").getDouble(0.0);
    }
    return cl;
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
