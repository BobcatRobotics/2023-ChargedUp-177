// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import frc.robot.utils.ModuleMap;
import java.util.Map;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final class USB {
    public static final int leftJoystick = 0;
    public static final int rightJoystick = 1;
    public static final int xBoxController = 2;
    public static final int testController = 4;
  }

  public static final class CAN {
    public static final int pigeon = 9;

    public static final int frontLeftCanCoder = 10;
    public static final int frontRightCanCoder = 11;
    public static final int backLeftCanCoder = 12;
    public static final int backRightCanCoder = 13;

    public static final int frontLeftDriveMotor = 20;
    public static final int frontLeftTurnMotor = 21;
    public static final int frontRightDriveMotor = 22;
    public static final int frontRightTurnMotor = 23;
    public static final int backLeftDriveMotor = 24;
    public static final int backLeftTurnMotor = 25;
    public static final int backRightDriveMotor = 26;
    public static final int backRightTurnMotor = 27;
  }

  public static final class Swerve {
    public static final double kTrackWidth = Units.inchesToMeters(30);
    public static final double kWheelBase = Units.inchesToMeters(30);

    public static final Map<ModulePosition, Translation2d> kModuleTranslations =
        Map.of(
            ModulePosition.FRONT_LEFT, new Translation2d(kWheelBase / 2, kTrackWidth / 2),
            ModulePosition.FRONT_RIGHT, new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
            ModulePosition.BACK_LEFT, new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
            ModulePosition.BACK_RIGHT, new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    public static final double frontLeftCANCoderOffset = 94.219;
    public static final double frontRightCANCoderOffset = 132.363;
    public static final double backLeftCANCoderOffset = 284.590;
    public static final double backRightCANCoderOffset = 179.648;

    public static final SwerveDriveKinematics kSwerveKinematics =
        new SwerveDriveKinematics(
            ModuleMap.orderedValues(kModuleTranslations, new Translation2d[0]));

    public static final double kMaxSpeedMetersPerSecond = Units.feetToMeters(18);
    public static final double kMaxRotationRadiansPerSecond = Math.PI * 2.0;
    public static final double kMaxRotationRadiansPerSecondSquared = Math.PI * 2.0;

    public static final double kP_X = 0.2;
    public static final double kD_X = 0;
    public static final double kP_Y = 0.2;
    public static final double kD_Y = 0;
    public static final double kP_Theta = 8;
    public static final double kD_Theta = 0;

    public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
        new TrapezoidProfile.Constraints(
            kMaxRotationRadiansPerSecond, kMaxRotationRadiansPerSecondSquared);

    public static final class Module {
      public static final double kDriveGearRatio = 6.12;
      public static final double kTurnGearRatio = 12.8;
      public static final double kWheelDiameterMeters = Units.inchesToMeters(3.94);
      public static final int kFalconEncoderCPR = 2048;
      public static final int kCANCoderCPR = 4096;

      public static final DCMotor kDriveGearbox = DCMotor.getFalcon500(1);
      public static final DCMotor kTurnGearbox = DCMotor.getFalcon500(1);

      public static final double kDriveDistancePerPulse =
          (kWheelDiameterMeters * Math.PI) / (kFalconEncoderCPR * kDriveGearRatio);
      public static final double kTurnDistancePerPulse =
          360.0 / (kFalconEncoderCPR * kTurnGearRatio);
      public static final double kTurningEncoderDistancePerPulse = 360.0 / kCANCoderCPR;

      public static final double ksDriveVoltSecondsPerMeter = 0.667 / 12;
      public static final double kvDriveVoltSecondsSquaredPerMeter = 2.44 / 12;
      public static final double kaDriveVoltSecondsSquaredPerMeter = 0.27 / 12;

      public static final double kvTurnVoltSecondsPerRadian = 1.47; // originally 1.5
      public static final double kaTurnVoltSecondsSquaredPerRadian = 0.348; // originally 0.3
    }
    
    public enum ModulePosition {
      FRONT_LEFT,
      FRONT_RIGHT,
      BACK_LEFT,
      BACK_RIGHT
    }
  }

  // 1 = closed-loop control (using sensor feedback) and 0 = open-loop control (no sensor feedback)
  public enum CONTROL_MODE {
    OPENLOOP,
    CLOSEDLOOP
  }
}
