// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }


  public static final int intakeMotorID = 11; // TODO: assign correct values
  public static final int wristSolenoidID = 0;
  public static final int pHubID = 1;
  public static final int compressorID = 1;
  public static final int wristCANCoderID = 999;//TODO: assign correct values
  public static final int wristMotorID = 9999;

  public static final class ArmConstants {
    public static final int armMotorPort = 10;
    public static final int stowedLimitSwitch = 2;

    // TODO: Change!
    public static final int pos0 = 0; // done
    public static final int pos1 = 16710; // done
    public static final int pos2 = 25658; // done
    public static final int minNonCollidingExtention = pos1 - 500;
    public static final int bottomPickup = 43664 - 300;

    public static final double bottomLimit = 0.0;
    public static final double constrictedBottomLimit = 4096;
    public static final double topLimit = 8192;

    // public static final int armlimitport = 2;

    public static int armState = 0;
  }

  public static class ElevatorConstants {
    public static final int elevatorMotorPort = 9;
    public static final int topLimitPort = 1;
    public static final int bottomLimitPort = 0;

    public static final int pos0 = 0; // done
    public static final int pos1 = -126256; // dpne
    public static final int pos2 = -226710; // done used to be 236000 not 226000

    public static final double topLimit = -236710;

    public static int elevatorState = 0;
  }



  public static final class WristConstants {
    public static final int forwardGround = 0; //TODO: Tune!
    public static final int topGround = 0;
    public static final int HPChute = 0;
    public static final int HPSlide = 0;
    public static final int score = 0;
    public static final int holdPiece = 0;

    public static final int topLimit = 0;
    public static final int bottomLimit = 0;
}

public static final class LEDConstants {
    public static final int ledPort = 0; // placeholder
}
}
