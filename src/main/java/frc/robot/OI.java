package frc.robot;

import java.nio.file.Path;
import java.time.DateTimeException;
import java.util.List;
import java.util.function.BiConsumer;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;

import edu.wpi.first.wpilibj2.command.button.InternalButton;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.subsystems.*;
import frc.robot.commands.*;
import frc.robot.RobotMap;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {


  // Joysticks
  public static Joystick rightStick = new Joystick(RobotMap.rightJoystick);
  public static Joystick leftStick = new Joystick(RobotMap.leftJoystick);
  public static Joystick gamePad = new Joystick(RobotMap.gamePad);

  public static JoystickButton btnDriveStraight = new JoystickButton(gamePad, 1);
  public static JoystickButton btnToggleLED = new JoystickButton(gamePad, 4);

}

  
  