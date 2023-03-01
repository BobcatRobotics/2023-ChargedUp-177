// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CAN;
import frc.robot.Constants.Swerve;
import frc.robot.Constants.Swerve.ModulePosition;
import frc.robot.utils.ModuleMap;

import java.util.HashMap;
import java.util.Map;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.Pigeon2;
import com.ctre.phoenix.unmanaged.Unmanaged;

public class SwerveDrive extends SubsystemBase {

  private final HashMap<ModulePosition, SwerveModule> m_swerveModules =
      new HashMap<>(
          Map.of(
              ModulePosition.FRONT_LEFT,
                  new SwerveModule(
                      ModulePosition.FRONT_LEFT,
                      new TalonFX(CAN.frontLeftTurnMotor),
                      new TalonFX(CAN.frontLeftDriveMotor),
                      new CANCoder(CAN.frontLeftCanCoder),
                      Swerve.frontLeftCANCoderOffset),
              ModulePosition.FRONT_RIGHT,
                  new SwerveModule(
                      ModulePosition.FRONT_RIGHT,
                      new TalonFX(CAN.frontRightTurnMotor),
                      new TalonFX(CAN.frontRightDriveMotor),
                      new CANCoder(CAN.frontRightCanCoder),
                      Swerve.frontRightCANCoderOffset),
              ModulePosition.BACK_LEFT,
                  new SwerveModule(
                      ModulePosition.BACK_LEFT,
                      new TalonFX(CAN.backLeftTurnMotor),
                      new TalonFX(CAN.backLeftDriveMotor),
                      new CANCoder(CAN.backLeftCanCoder),
                      Swerve.backLeftCANCoderOffset),
              ModulePosition.BACK_RIGHT,
                  new SwerveModule(
                      ModulePosition.BACK_RIGHT,
                      new TalonFX(CAN.backRightTurnMotor),
                      new TalonFX(CAN.backRightDriveMotor),
                      new CANCoder(CAN.backRightCanCoder),
                      Swerve.backRightCANCoderOffset)));

  private final Pigeon2 m_pigeon = new Pigeon2(CAN.pigeon);

  private final SwerveDriveOdometry m_odometry =
      new SwerveDriveOdometry(
          Swerve.kSwerveKinematics,
          getHeadingRotation2d(),
          getModulePositions(),
          new Pose2d());

  private PIDController m_xController = new PIDController(Swerve.kP_X, 0, Swerve.kD_X);
  private PIDController m_yController = new PIDController(Swerve.kP_Y, 0, Swerve.kD_Y);
  private ProfiledPIDController m_turnController =
      new ProfiledPIDController(Swerve.kP_Theta, 0, Swerve.kD_Theta, Swerve.kThetaControllerConstraints);

  private double m_simYaw;

  public SwerveDrive() {
    m_pigeon.setYaw(0);
  }

  public void drive(
      double throttle,
      double strafe,
      double rotation,
      boolean isFieldRelative,
      boolean isOpenLoop) {
    throttle *= Swerve.kMaxSpeedMetersPerSecond;
    strafe *= Swerve.kMaxSpeedMetersPerSecond;
    rotation *= Swerve.kMaxRotationRadiansPerSecond;

    ChassisSpeeds chassisSpeeds =
        isFieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(
                throttle, strafe, rotation, getHeadingRotation2d())
            : new ChassisSpeeds(throttle, strafe, rotation);

    Map<ModulePosition, SwerveModuleState> moduleStates =
        ModuleMap.of(Swerve.kSwerveKinematics.toSwerveModuleStates(chassisSpeeds));

    SwerveDriveKinematics.desaturateWheelSpeeds(
        ModuleMap.orderedValues(moduleStates, new SwerveModuleState[0]), Swerve.kMaxSpeedMetersPerSecond);

    for (SwerveModule module : ModuleMap.orderedValuesList(m_swerveModules))
      module.setDesiredState(moduleStates.get(module.getModulePosition()), isOpenLoop);
  }

  public void setSwerveModuleStates(SwerveModuleState[] states, boolean isOpenLoop) {
    SwerveDriveKinematics.desaturateWheelSpeeds(states, Swerve.kMaxSpeedMetersPerSecond);

    for (SwerveModule module : ModuleMap.orderedValuesList(m_swerveModules))
      module.setDesiredState(states[module.getModulePosition().ordinal()], isOpenLoop);
  }

  public void setSwerveModuleStatesAuto(SwerveModuleState[] states) {
    setSwerveModuleStates(states, false);
  }

  public void resetModsToAbsolute() {
    for (SwerveModule module : ModuleMap.orderedValuesList(m_swerveModules)) {
      module.resetAngleToAbsolute();
    }
  }

  public void setOdometry(Pose2d pose) {
    m_odometry.resetPosition(getHeadingRotation2d(), getModulePositions(), pose);
    m_pigeon.setYaw(pose.getRotation().getDegrees());
  }

  public double getHeadingDegrees() {
    return Math.IEEEremainder(m_pigeon.getYaw(), 360);
  }

  public Rotation2d getHeadingRotation2d() {
    return Rotation2d.fromDegrees(getHeadingDegrees());
  }

  public Pose2d getPoseMeters() {
    return m_odometry.getPoseMeters();
  }

  public SwerveModule getSwerveModule(ModulePosition modulePosition) {
    return m_swerveModules.get(modulePosition);
  }

  public Map<ModulePosition, SwerveModuleState> getModuleStates() {
    Map<ModulePosition, SwerveModuleState> map = new HashMap<>();
    for (ModulePosition i : m_swerveModules.keySet()) {
      map.put(i, m_swerveModules.get(i).getState());
    }
    return map;
  }

  public SwerveModulePosition[] getModulePositions() {
    return new SwerveModulePosition[] {
        m_swerveModules.get(ModulePosition.FRONT_LEFT).getPosition(),
        m_swerveModules.get(ModulePosition.FRONT_RIGHT).getPosition(),
        m_swerveModules.get(ModulePosition.BACK_LEFT).getPosition(),
        m_swerveModules.get(ModulePosition.BACK_RIGHT).getPosition()
    };
  }

  public PIDController getXPidController() {
    return m_xController;
  }

  public PIDController getYPidController() {
    return m_yController;
  }

  public ProfiledPIDController getThetaPidController() {
    return m_turnController;
  }

  public void setNeutralMode(NeutralMode mode) {
    for (SwerveModule module : m_swerveModules.values()) {
      module.setDriveNeutralMode(mode);
      module.setTurnNeutralMode(mode);
    }
  }

  public SwerveDriveOdometry getOdometry() {
    return m_odometry;
  }

  public void updateOdometry() {
    m_odometry.update(
        getHeadingRotation2d(),
        getModulePositions());

    for (SwerveModule module : ModuleMap.orderedValuesList(m_swerveModules)) {
      Translation2d modulePositionFromChassis =
          Swerve.kModuleTranslations
              .get(module.getModulePosition())
              .rotateBy(getHeadingRotation2d())
              .plus(getPoseMeters().getTranslation());
      module.setModulePose(
          new Pose2d(
              modulePositionFromChassis,
              module.getHeadingRotation2d().plus(getHeadingRotation2d())));
    }
  }

  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.kMaxSpeedMetersPerSecond);
    
    for(SwerveModule mod : m_swerveModules.values()){
        mod.setDesiredState(desiredStates[mod.m_moduleNumber], false);
    }
}

  private void updateSmartDashboard() {}

  @Override
  public void periodic() {
    updateOdometry();
    updateSmartDashboard();
  }

  @Override
  public void simulationPeriodic() {
    ChassisSpeeds chassisSpeed =
        Swerve.kSwerveKinematics.toChassisSpeeds(
            ModuleMap.orderedValues(getModuleStates(), new SwerveModuleState[0]));

    m_simYaw += chassisSpeed.omegaRadiansPerSecond * 0.02;

    Unmanaged.feedEnable(20);
    m_pigeon.getSimCollection().setRawHeading(-Units.radiansToDegrees(m_simYaw));
  }
}
