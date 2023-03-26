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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class PoseEstimator extends SubsystemBase {
  private static final Vector<N3> stateStdDevs = VecBuilder.fill(0.1, 0.1, 0.1);
  private static final Vector<N3> visionMeasurementStdDevs = VecBuilder.fill(0.9, 0.9, Units.degreesToRadians(0.9));

  private final Supplier<Rotation2d> rotationSupplier;
  private final Supplier<SwerveModulePosition[]> modulePositionSupplier;
  private final SwerveDrivePoseEstimator poseEstimator;
  //private PhotonVision photon = new PhotonVision();
  private Limelight limelight;

  private OriginPosition originPosition = OriginPosition.kBlueAllianceWallRightSide;
  private boolean sawTag = false;

  private final double FIELD_LENGTH_METERS = 16.54175;
  private final double FIELD_WIDTH_METERS = 8.0137;
  
  private Alliance alliance = Alliance.Blue;

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
    this.limelight.setAlliance(alliance);
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

  /**
   * 
   * @return grid #
   * 1 - closest to hp station
   * 2 - middle grid
   * 3 - furthest from hp station
   */
  public static int closestGrid(Pose2d currPos){
    double currX = currPos.getX();
    double currY = currPos.getY();
    

    // find the distance from the supplied pose to the middle of each grid
    // 1 - middle of grid furthest from HP station, 2 - middle of middle grid, 3 - middle of grid closest to HP station
    double distTo1 = distanceFormula(currX, currY, Constants.PoseEstimation.grid1[1].getX(), Constants.PoseEstimation.grid1[1].getY());
    double distTo2 = distanceFormula(currX, currY, Constants.PoseEstimation.grid2[1].getX(), Constants.PoseEstimation.grid2[1].getY());
    double distTo3= distanceFormula(currX, currY, Constants.PoseEstimation.grid3[1].getX(), Constants.PoseEstimation.grid3[1].getY());
    //               grid 1   grid 2   grid 3
    double[] vals = {distTo1, distTo2, distTo3};
    
    // find the smallest distance
    int minIndex = 4;
    for(int i = 0; i < vals.length; i++){
      if(vals[i] < minIndex){
        minIndex = i + 1;
      }
    }

    //ofset index by 1
    return minIndex;
  }

  public static double distanceFormula(double x1, double y1, double x2, double y2){
    return Math.sqrt(
      Math.pow((x2-x1), 2)  -  (Math.pow((y2-y1), 2))
      );
  }
   
  @Override
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
      double[] botPose = limelight.botPose();
      Rotation3d rot3 = new Rotation3d(Units.degreesToRadians(botPose[3]), Units.degreesToRadians(botPose[4]), Units.degreesToRadians(botPose[5]));
  
      visionPose = new Pose3d(botPose[0], botPose[1], botPose[2], rot3);
    } catch (Exception e) {
      System.out.println(e);
    }
    if (visionPose != null && limelight.tv() == 1.0) {
      System.out.println("vision pose found!");
      sawTag = true;
      
      Pose2d pose2d = new Pose2d(visionPose.getTranslation().toTranslation2d(), rotationSupplier.get());

      double distance = limelight.targetDist();
      double timeStampSeconds =  Timer.getFPGATimestamp() - (limelight.tl()/1000.0) - (limelight.cl()/1000.0);
      // double poseDist = distanceFormula(pose2d.getX(),pose2d.getY(),visionPose.getX(),visionPose.getY());
      if (distanceFormula(pose2d.getX(),pose2d.getY(),visionPose.getX(),visionPose.getY()) < 1.0) {
        poseEstimator.addVisionMeasurement(pose2d, timeStampSeconds, VecBuilder.fill(distance/2, distance/2, 100));
      }
      //setCurrentPose(pose2d);
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