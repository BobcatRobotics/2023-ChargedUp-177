package frc.robot;

import BobcatLib.Subsystems.Swerve.SimpleSwerve.Containers.SwerveBase;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.*;
import frc.robot.commands.Autos.AlignToTarget;
import frc.robot.commands.Autos.BalanceChargeStation;
import frc.robot.commands.Presets.RetractArm;
import frc.robot.commands.Presets.RunIntake;
import frc.robot.commands.Presets.SetArm;
import frc.robot.commands.Presets.SetElevator;
import frc.robot.commands.Presets.StartingConfig;
import frc.robot.commands.Presets.ZeroElevator;
import frc.robot.commands.Presets.intakeStop;
import frc.robot.commands.Presets.Procedures.ForwardSuck;
import frc.robot.commands.Presets.Procedures.ScoreHigh;
import frc.robot.commands.Presets.Procedures.ScoreMid;
import frc.robot.commands.Presets.Procedures.TopSuck;
import frc.robot.subsystems.*;

import java.util.List;

import BobcatLib.Hardware.Controllers.OI;
import BobcatLib.Subsystems.Swerve.SimpleSwerve.Swerve.Module.Utility.PIDConstants;
import BobcatLib.Subsystems.Swerve.SimpleSwerve.Utility.Alliance;
import BobcatLib.Subsystems.Swerve.Utility.LoadablePathPlannerAuto;
import frc.robot.Constants.Swerve;
import frc.robot.autos.RedHighCone6PickupBalance;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer extends SwerveBase {
    // for the xbox controller buttons
    // private Constants.ButtonHashtable bh = new Constants.ButtonHashtable();

    /* Controllers */
    private final Joystick driver = new Joystick(2);
    private final Joystick rotate = new Joystick(0);
    private final Joystick strafe = new Joystick(1);

    // /* Drive Controls */
    // private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    /* Driver Buttons */

    private final JoystickButton xButton = new JoystickButton(driver, 5);
    // private final JoystickButton alignRobot = new JoystickButton(driver, 2);
    // private final JoystickButton zeroGyro = new JoystickButton(driver, 4);

    // ruffy buttons
    private final JoystickButton ruffy0 = new JoystickButton(rotate, 1);
    private final JoystickButton ruffy1 = new JoystickButton(strafe, 1);

    // private final JoystickButton alignRobot = new JoystickButton(driver, 1);

    private final JoystickButton leftBumper = new JoystickButton(driver, 5); // left bumper
    private final JoystickButton rightBumper = new JoystickButton(driver, 6);// right bumper
    private final JoystickButton a = new JoystickButton(driver, 2);
    private final JoystickButton b = new JoystickButton(driver, 3);
    private final JoystickButton righttrigger = new JoystickButton(driver, 8);
    private final JoystickButton lefttrigger = new JoystickButton(driver, 7);
    private final JoystickButton back = new JoystickButton(driver, 9);
    private final POVButton DUp = new POVButton(driver, 0);
    private final POVButton DLeft = new POVButton(driver, 270);
    private final POVButton DDown = new POVButton(driver, 180);
    /* Subsystems */
    public static Limelight m_Limelight = new Limelight();

    private final Elevator m_Elevator = new Elevator();
    private final Intake m_Intake = new Intake();
    private final Arm m_Arm = new Arm();
    private final Wrist m_Wrist = new Wrist();

    /* Commands */
    private final Command elevatorControls = new ElevatorControls(m_Elevator, driver, m_Arm);
    private final Command armControls = new ArmControls(m_Arm, driver, m_Elevator);
    private final Command setarm0 = new SetArm(m_Arm, 0);
    private final Command setarm1 = new SetArm(m_Arm, 1);
    private final Command setarm2 = new SetArm(m_Arm, 2);
    private final Command setelevator0 = new SetElevator(m_Elevator, 0);
    private final Command setelevator2 = new SetElevator(m_Elevator, 2);
    private final Command align;
    private final RedHighCone6PickupBalance redHighCone6PickupBalance = new RedHighCone6PickupBalance(s_Swerve,m_Limelight);

    /* SendableChooser */
    SendableChooser<SequentialCommandGroup> autoChooser = new SendableChooser<>();

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer(OI driver_controller,
            List<LoadablePathPlannerAuto> autos,
            String robotName,
            boolean isSim,
            Alliance alliance,
            PIDConstants tranPidPathPlanner,
            PIDConstants rotPidPathPlanner) {

        super(driver_controller, autos, robotName, isSim, alliance, tranPidPathPlanner, rotPidPathPlanner);

        align = new AlignToTarget(super.s_Swerve, m_Limelight);
    }

    // returns the angle of the joystick in degrees
    public double getJoystickAngle() {
        return ((Math.atan2(rotate.getRawAxis(Joystick.AxisType.kY.value),
                rotate.getRawAxis(Joystick.AxisType.kX.value)) * 180 / Math.PI) + 360) % 360;
    }

    public void displayGyro() {
        SmartDashboard.putNumber("pitch", super.s_Swerve.getBaseGyro().getPitch().getDegrees());
        SmartDashboard.putNumber("yaw", super.s_Swerve.getBaseGyro().getRoll().getDegrees());
    }

    public Command getDefaultCommand() {
        return s_Swerve.getDefaultCommand();
    }

    public void resetToAbsolute() {
        s_Swerve.resetModulesToAbsolute();
    }

    public void scheduleDefaultTeleop() {
        // Sendable Chooser Setup
        SmartDashboard.putData(autoChooser);
        Constants.AutoConstants.eventMap.put("chargeStation", align);
        m_Arm.setDefaultCommand(armControls);
        m_Elevator.setDefaultCommand(elevatorControls);
        m_Intake.setDefaultCommand(new RunIntake(m_Intake, driver));

        // Configure the button bindings
        configureButtonBindings();
        m_Limelight.initializeLimeLight();
    }

    public void cancelDefaultTeleop() {
        m_Arm.getDefaultCommand().cancel();
        m_Elevator.getDefaultCommand().cancel();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
     * it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    @Override
    public void configureButtonBindings() {
        super.configureButtonBindings();
        /* Driver Buttons */
        ruffy0.onTrue(new InstantCommand(() -> s_Swerve.zeroHeading()));
        ruffy1.onTrue(new InstantCommand(() -> s_Swerve.zeroHeading()));
        righttrigger.onTrue(new InstantCommand(m_Wrist::wristSolenoidOFF));
        rightBumper.onTrue(new InstantCommand(m_Wrist::wristSolenoidON));
        DUp.onTrue(new ScoreHigh(m_Elevator, m_Arm, m_Intake, m_Wrist).until(this::anythingPressed));
        DDown.onTrue(new StartingConfig(m_Elevator, m_Arm, m_Wrist).until(this::anythingPressed));
        DLeft.onTrue(new ScoreMid(m_Elevator, m_Arm, m_Intake, m_Wrist).until(this::anythingPressed));
        a.onTrue(new ForwardSuck(m_Elevator, m_Arm, m_Intake, m_Wrist).until(this::anythingPressed));
        b.onTrue(new TopSuck(m_Elevator, m_Arm, m_Intake, m_Wrist).until(this::anythingPressed));
    }

    public boolean anythingPressed() {
        return Math.abs(driver.getRawAxis(1)) >= 0.1 || Math.abs(driver.getRawAxis(3)) >= 0.1;
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return new InstantCommand();
    }
}
