package frc.robot;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.Constants;
import frc.robot.Constants.ButtonHashtable;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.autos.exampleAuto;
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
    //for the xbox controller buttons
    private Constants.ButtonHashtable bh = new Constants.ButtonHashtable();

    /* Controllers */
    private final Joystick driver = new Joystick(2);
    private final Joystick rotate = new Joystick(0);
    private final Joystick strafe = new Joystick(1);

    // /* Drive Controls */
    // private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, 4);
    //private final JoystickButton robotCentric = new JoystickButton(driver, 5);
    private final JoystickButton alignRobot = new JoystickButton(driver, 1);


    private final JoystickButton leftBumper = new JoystickButton(driver, 5); //left bumper
    private final JoystickButton rightBumper = new JoystickButton(driver, 6);//right bumper
    private final JoystickButton a = new JoystickButton(driver, 2);
    private final JoystickButton b = new JoystickButton(driver, 3);
    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();

    private final Elevator m_Elevator = new Elevator();
    private final Intake m_Intake = new Intake();
    private final Arm m_Arm = new Arm();
    private final Wrist m_Wrist = new Wrist();
    
    /* Commands */
    private final Command elevatorControls = new ElevatorControls(m_Elevator, driver);
    private final Command armControls = new ArmControls(m_Arm, driver);
    private final Limelight m_Limelight = new Limelight();
    //private final Command align = new AlignToTarget(s_Swerve, m_Limelight).withInterruptBehavior(InterruptionBehavior.kCancelIncoming).repeatedly();
    private final Command align = new AlignToTarget(s_Swerve, m_Limelight);


    public double getJoystickAngle(){
        return  ((Math.atan2(rotate.getRawAxis(Joystick.AxisType.kY.value), rotate.getRawAxis(Joystick.AxisType.kX.value)) * 180 / Math.PI)+360)%360;
    }
    public void displayGyro(){
        SmartDashboard.putNumber("pitch", s_Swerve.getPitch());
        SmartDashboard.putNumber("yaw", s_Swerve.getRoll());
    }
    // returns the angle of the joystick in degrees
   
    public Command getDefaultCommand(){
        return s_Swerve.getDefaultCommand();
    }
    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        /*
         * Default commands should be scheduled here if they should run all the time (auto and teleop).
         * Make sure that none of the default commands require the same subsystems that you intend to
         * use during autonomous, because they will interrupt the autonomous command. If you want a default
         * command to run just during teleop, schedule it in the scheduleDefaultTeleop method and cancel
         * it in the cancelDefaultTeleop method.
         */

        // Configure the button bindings
        configureButtonBindings();
    }

    public void scheduleDefaultTeleop() {
        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> -strafe.getRawAxis(Joystick.AxisType.kY.value)*Math.abs(strafe.getRawAxis(Joystick.AxisType.kY.value)), 
                () -> -strafe.getRawAxis(Joystick.AxisType.kX.value)*Math.abs(strafe.getRawAxis(Joystick.AxisType.kX.value)), 
                () -> -rotate.getRawAxis(Joystick.AxisType.kX.value), 
                () -> true //() -> robotCentric.getAsBoolean()
            )
        );



        m_Arm.setDefaultCommand(armControls);
        m_Elevator.setDefaultCommand(elevatorControls);

        // Configure the button bindings
        configureButtonBindings();
        m_Limelight.initializeLimeLight();
    }

    public void cancelDefaultTeleop() {
        s_Swerve.getDefaultCommand().cancel();
        m_Arm.getDefaultCommand().cancel();
        m_Elevator.getDefaultCommand().cancel();
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

        leftBumper.whileTrue((new RunCommand(m_Intake::runIntakeIn).andThen(new InstantCommand(m_Intake::stop))));
        rightBumper.whileTrue((new RunCommand(m_Intake::runIntakeOut).andThen(new InstantCommand(m_Intake::stop))));
        rightBumper.whileFalse((new InstantCommand(() -> m_Intake.stop())));
        leftBumper.whileFalse((new InstantCommand(() -> m_Intake.stop())));
        a.onTrue(new InstantCommand(m_Wrist::wristSolenoidOFF));
        b.onTrue(new InstantCommand(m_Wrist::wristSolenoidON));

        alignRobot.whileTrue(align);
    }

    public SequentialCommandGroup MountAndBalance(Swerve s_Swerve){
        // get on the charge station, then balance, then put the wheels in an x configuration
        return new SequentialCommandGroup(
            new MountChargeStation(s_Swerve, false),
            new BalanceChargeStation(s_Swerve, false),
            new InstantCommand(() -> s_Swerve.configToX())
        );

    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        s_Swerve.resetOdometry(new Pose2d(0, 0, s_Swerve.getYaw()));
        return new exampleAuto(s_Swerve, m_Limelight);
    }
}
