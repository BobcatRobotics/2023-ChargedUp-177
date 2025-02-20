package frc.robot;

import java.util.HashMap;
import java.util.Hashtable;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.util.SwerveModuleConstants;

public final class Constants {
    public static final int intakeMotorID = 11; //TODO: assign correct values
    public static final int wristSolenoidID = 0;
    public static final int wristMotorID = 0;
    public static final int pHubID = 1;
    public static final int compressorID = 1;
    public static final double stickDeadband = 0;
    public static final int Jake = 194; //Jake M is 194 cm tall

    public static final class Swerve {
        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 { //TODO: This must be tuned to specific robot

            public static final int driveMotorID = 1;
            public static final int angleMotorID = 2;
            public static final int canCoderID = 1;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(174.3);
            
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 { //TODO: This must be tuned to specific robot

            public static final int driveMotorID = 3;
            public static final int angleMotorID = 4;
            public static final int canCoderID = 2;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(188.61);

            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
        
        /* Back Left Module - Module 2 */
        public static final class Mod2 { //TODO: This must be tuned to specific robot

            public static final int driveMotorID = 5;
            public static final int angleMotorID = 6;
            public static final int canCoderID = 3;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(48.52);

            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 { //TODO: This must be tuned to specific robot

            public static final int driveMotorID = 7;
            public static final int angleMotorID = 8;
            public static final int canCoderID = 4;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(104.677);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
    }

    public static final class AutoConstants { //TODO: The below constants are used in the example auto, and must be tuned to specific robot
        public static final double kMaxSpeedMetersPerSecond = 1;
        public static final double kMaxAccelerationMetersPerSecondSquared = 1;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
    
        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 1;

        public static HashMap<String, Command> eventMap = new HashMap<>();
    
        /* Constraint for the motion profilied robot angle controller */
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }
    public class BalancingConstants{
        public static final double kP = 0.3;//TODO: tune
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

    public static final class FieldConstants {
        public static final double length = 10;
        public static final double width = 10;
    }

    public static final class ArmConstants {
        public static final int armMotorPort = 10; 
        public static final int stowedLimitSwitch = 2;

        // TODO: Change!
        public static final int pos0 = 0; // done
        public static final int pos1 = 16710; // done
        public static final int pos2 = 25658; // done
        public static final int minNonCollidingExtention = pos1-500;
        public static final int bottomPickup = 43664 - 300;

        public static final double bottomLimit = 0.0;
        public static final double constrictedBottomLimit = 4096;
        public static final double topLimit = 8192;

        //public static final int armlimitport = 2;

        public static int armState = 0;
    }

    public static final class ElevatorConstants {
        public static final int elevatorMotorPort = 9;
        public static final int topLimitPort = 1;
        public static final int bottomLimitPort = 0;

        public static final int pos0 = 0; // done
        public static final int pos1 = -126256; // dpne
        public static final int pos2 = -226710; // done used to be 236000 not 226000

        public static final double topLimit = -236710;

        public static int elevatorState = 0;
    }
}
