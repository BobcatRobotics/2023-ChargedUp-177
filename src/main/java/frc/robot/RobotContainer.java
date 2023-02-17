package frc.robot;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.autos.PathPlannerTest;
import frc.robot.autos.RedHighCone6PickupBalance;
import frc.robot.commands.AlignToTarget;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Swerve;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    private final Joystick driver = new Joystick(2);
    private final Joystick rotate = new Joystick(0);
    private final Joystick strafe = new Joystick(1);

    // /* Drive Controls */
    // private final int translationAxis = XboxController.Axis.kLeftY.value;
    // private final int strafeAxis = XboxController.Axis.kLeftX.value;
    // private final int rotationAxis = XboxController.Axis.kRightX.value;

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, 4);
    private final JoystickButton resetToAbsolute = new JoystickButton(driver, 3);
    private final JoystickButton robotCentric = new JoystickButton(driver, 5);
    private final JoystickButton alignRobot = new JoystickButton(driver, 2);

    /* Subsystems */
    public static Swerve s_Swerve = new Swerve();
    public static Limelight m_Limelight = new Limelight();

    /* Commands */
    //private final Command align = new AlignToTarget(s_Swerve, m_Limelight).withInterruptBehavior(InterruptionBehavior.kCancelIncoming).repeatedly();
    private final Command align = new AlignToTarget(s_Swerve, m_Limelight);
    private final RedHighCone6PickupBalance redHighCone6PickupBalance = new RedHighCone6PickupBalance(s_Swerve, m_Limelight);
    private final PathPlannerTest pathPlannerTest = new PathPlannerTest();

    /* SendableChooser */
    SendableChooser<SequentialCommandGroup> autoChooser = new SendableChooser<>();

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> -strafe.getRawAxis(Joystick.AxisType.kY.value)*Math.abs(strafe.getRawAxis(Joystick.AxisType.kY.value)), 
                () -> -strafe.getRawAxis(Joystick.AxisType.kX.value)*Math.abs(strafe.getRawAxis(Joystick.AxisType.kX.value)), 
                () -> -rotate.getRawAxis(Joystick.AxisType.kX.value), 
                () -> robotCentric.getAsBoolean()
            )
        );

        // Sendable Chooser Setup
        autoChooser.setDefaultOption("Red High Cone 6 Pickup & Balance", redHighCone6PickupBalance);
        autoChooser.addOption("PathPlanner Test", pathPlannerTest);
        SmartDashboard.putData(autoChooser);
        // Configure the button bindings
        configureButtonBindings();
        m_Limelight.initializeLimeLight();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */
        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));
        resetToAbsolute.onTrue(new InstantCommand(() -> s_Swerve.resetModulesToAbsolute()));
        alignRobot.whileTrue(align.andThen(new InstantCommand(() -> SmartDashboard.putBoolean("alignpressed", true))));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        s_Swerve.resetOdometry(new Pose2d(0, 0, s_Swerve.getYaw()));
        return autoChooser.getSelected();
    }
}
