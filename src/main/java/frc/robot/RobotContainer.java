package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    private final CommandXboxController driver = new CommandXboxController(2);
    private final Joystick rotate = new Joystick(0);
    private final Joystick strafe = new Joystick(1);

    // /* Drive Controls */
    // private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    /* Driver Buttons */
    private final Trigger zeroGyro = driver.y();
    private final Trigger robotCentric = driver.leftBumper();

    /* Subsystems */
    //private final Swerve s_Swerve = new Swerve();
    private final Intake m_Intake = new Intake();
    private final Wrist m_Wrist = new Wrist();

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {

        // Configure the button bindings
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */
        //zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));
        driver.leftBumper().whileTrue((new RunCommand(m_Intake::runIntakeIn).andThen(new InstantCommand(m_Intake::stop))));
        driver.rightBumper().whileTrue((new RunCommand(m_Intake::runIntakeOut).andThen(new InstantCommand(m_Intake::stop))));
        driver.rightBumper().whileFalse((new InstantCommand(() -> m_Intake.stop())));
        driver.leftBumper().whileFalse((new InstantCommand(() -> m_Intake.stop())));
        driver.a().onTrue(new InstantCommand(m_Wrist::wristSolenoidOFF));
        driver.b().onTrue(new InstantCommand(m_Wrist::wristSolenoidON));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
  //  public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        //s_Swerve.resetOdometry(new Pose2d(0, 0, s_Swerve.getYaw()));
        //return new exampleAuto(s_Swerve);
    //}
}
