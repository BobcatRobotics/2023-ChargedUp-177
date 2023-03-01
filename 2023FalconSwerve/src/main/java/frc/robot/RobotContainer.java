// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.SetSwerveDrive;
import frc.robot.simulation.FieldSim;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.Constants.USB;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public static final SwerveDrive m_robotDrive = new SwerveDrive();

  private final FieldSim m_fieldSim = new FieldSim(m_robotDrive);

  static Joystick leftJoystick = new Joystick(USB.leftJoystick);
  static Joystick rightJoystick = new Joystick(USB.rightJoystick);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    // Configure default commands
    m_robotDrive.setDefaultCommand(
            // The left stick controls translation of the robot.
            // Turning is controlled by the X axis of the right stick.
            new SetSwerveDrive(
                    m_robotDrive,
                    ()-> leftJoystick.getY(),
                    ()-> leftJoystick.getX(),
                    ()-> rightJoystick.getX(),
                    false));

    m_fieldSim.initSim();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    HashMap<String, Command> eventMap = new HashMap<>();
    //eventMap.put("wait", new WaitCommand(10));

    SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
      m_robotDrive::getPoseMeters, 
      m_robotDrive::setOdometry, 
      Constants.Swerve.kSwerveKinematics, 
      new PIDConstants(1, 0, 0),
      new PIDConstants(1, 0, 0),
      m_robotDrive::setSwerveModuleStatesAuto,
      eventMap, 
      true, 
      m_robotDrive
    );

    return autoBuilder.fullAuto(PathPlanner.loadPathGroup("New Path", 4, 4));
  }
  
  public void periodic() {
    m_fieldSim.periodic();
  }
}
