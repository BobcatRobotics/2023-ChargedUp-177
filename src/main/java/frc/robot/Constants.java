package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color;
import frc.lib.util.COTSFalconSwerveConstants;
import frc.lib.util.SwerveModuleConstants;
import java.util.Hashtable;

public final class Constants {
    public static final double stickDeadband = 0;

    public static final class Swerve {
        public static final int pigeonID = 1;
        public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-

        public static final COTSFalconSwerveConstants chosenModule =  //TODO: This must be tuned to specific robot
            COTSFalconSwerveConstants.SDSMK4i(COTSFalconSwerveConstants.driveGearRatios.SDSMK4i_L3);

        /* Drivetrain Constants */
        public static final double trackWidth = 0.476; // meters
        public static final double wheelBase = 0.476; // meters
        public static final double wheelCircumference = chosenModule.wheelCircumference;

        /* Swerve Kinematics 
         * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
         public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

        /* Module Gear Ratios */
        public static final double driveGearRatio = chosenModule.driveGearRatio;
        public static final double angleGearRatio = chosenModule.angleGearRatio;

        /* Motor Inverts */
        public static final boolean angleMotorInvert = chosenModule.angleMotorInvert;
        public static final boolean driveMotorInvert = chosenModule.driveMotorInvert;

        /* Angle Encoder Invert */
        public static final boolean canCoderInvert = chosenModule.canCoderInvert;

        /* Swerve Current Limiting */
        public static final int angleContinuousCurrentLimit = 25;
        public static final int anglePeakCurrentLimit = 40;
        public static final double anglePeakCurrentDuration = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveContinuousCurrentLimit = 35;
        public static final int drivePeakCurrentLimit = 60;
        public static final double drivePeakCurrentDuration = 0.1;
        public static final boolean driveEnableCurrentLimit = true;

        /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;

        /* Angle Motor PID Values */
        public static final double angleKP = chosenModule.angleKP;
        public static final double angleKI = chosenModule.angleKI;
        public static final double angleKD = chosenModule.angleKD;
        public static final double angleKF = chosenModule.angleKF;

        /* Drive Motor PID Values */
        public static final double driveKP = 0.05; //TODO: This must be tuned to specific robot
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.0;

        /* Drive Motor Characterization Values 
         * Divide SYSID values by 12 to convert from volts to percent output for CTRE */
        public static final double driveKS = (0.47309 / 12); // TUNED
        public static final double driveKV = (0.66261 / 12);
        public static final double driveKA = (0.071697 / 12);

        /* Swerve Profiling Values */
        /** Meters per Second */
        public static final double maxSpeed = 4.5; //TODO: This must be tuned to specific robot
        /** Radians per Second */
        public static final double maxAngularVelocity = 10.0; //TODO: This must be tuned to specific robot

        /* Neutral Modes */
        public static final NeutralMode angleNeutralMode = NeutralMode.Coast;
        public static final NeutralMode driveNeutralMode = NeutralMode.Brake;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 1;
            public static final int angleMotorID = 2;
            public static final int canCoderID = 1;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(170.7);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 3;
            public static final int angleMotorID = 4;
            public static final int canCoderID = 2;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(187.6);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
        
        /* Back Left Module - Module 2 */
        public static final class Mod2 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 5;
            public static final int angleMotorID = 6;
            public static final int canCoderID = 3;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(57.12);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 7;
            public static final int angleMotorID = 8;
            public static final int canCoderID = 4;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(105.29);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
    }

    public static final class AutoConstants { //TODO: The below constants are used in the example auto, and must be tuned to specific robot
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
    
        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 1;
    
        /* Constraint for the motion profilied robot angle controller */
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }
    public class BalancingConstants{
        public static final double kP = 0.4;//TODO: tune
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kToleranceDegrees = 2.5;//acceptable absolute error in degrees
        public static final double kSetpoint = 0.0; // we want a pitch of 0 degrees
        public static final double kSensitivity = 20; // sigmoid(pid/sensitivity)*max speed = meters per second to drive 
        
    }
    public static class ButtonHashtable {
        //hashtable used here to make use of key - value pairs
        //access through table.get(key);
        //add or change through table.put(key, value);
        //remove through table.remove(key);

        //If the types specified in the <>'s of the "New Hashtable<>" are the same as the first, you don't have to include them
        //Be sure to use WRAPPER CLASSES for primitive types
        public Hashtable<String, Integer> buttons = new Hashtable<>();
        public ButtonHashtable () {
            buttons.put("X_Button", 1);
            buttons.put("A_Button", 2);
            buttons.put("B_Button", 3);
            buttons.put("Y_Button", 4);
            
            buttons.put("Left_Bumper_Button", 5);
            buttons.put("Right_Bumper_Button", 6);
            buttons.put("Left_Trigger_Button", 7);
            buttons.put("Right_Trigger_Button", 8);

            //for some reason there are some variables in 2022 rapid react also
            //with 1 and 0

            buttons.put("D_Pad_Up", 0);
            buttons.put("D_Pad_Right", 90);
            buttons.put("D_Pad_Down", 180);
            buttons.put("D_Pad_Left", 2700);
        }
        
    }
    public static class ColorHashtable {
        //guide to hashtables in ButtonHashtable
        public Hashtable<String, Double> colors = new Hashtable<>();
        public ColorHashtable () {
            //TODO: add patterns and not just solid colors
            //possibly make a var and add .02 to it
            //in every argument so it is higher
            //for the next time it is called
            colors.put("Hot Pink", 0.57);
            colors.put("Dark Red", 0.59);
            colors.put("Red", 0.61);
            colors.put("Red Orange", 0.63);
            colors.put("Orange", 0.65);
            colors.put("Gold", 0.67);
            colors.put("Yellow", 0.69);
            colors.put("Lawn Green", 0.71);
            colors.put("Lime", 0.73);
            colors.put("Dark Green", 0.75);
            colors.put("Green", 0.77);
            colors.put("Blue Green", 0.79);
            colors.put("Aqua", 0.81);
            colors.put("Sky Blue", 0.83);
            colors.put("Dark Blue", 0.85);
            colors.put("Blue", 0.87);
            colors.put("Blue Violet", 0.89);
            colors.put("Violet", 0.91);
            colors.put("White", 0.93);
            colors.put("Gray", 0.95);
            colors.put("Dark Gray", 0.97);
            colors.put("Black", 0.99);
            
        }
    }
    
}
