package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.Constants;
import frc.robot.Constants.ButtonHashtable;

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
    // private final int strafeAxis = XboxController.Axis.kLeftX.value;
    // private final int rotationAxis = XboxController.Axis.kRightX.value;

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, bh.buttons.get("Y_Button"));
    private final JoystickButton robotCentric = new JoystickButton(driver, bh.buttons.get("Left_Bumper_Button"));

    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();

    public double getJoystickAngle(){
        return  ((Math.atan2(rotate.getRawAxis(Joystick.AxisType.kY.value), rotate.getRawAxis(Joystick.AxisType.kX.value)) * 180 / Math.PI)+360)%360;
    }

    // returns the angle of the joystick in degrees
   

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
                () -> -strafe.getRawAxis(Joystick.AxisType.kY.value), 
                () -> -strafe.getRawAxis(Joystick.AxisType.kX.value), 
                () -> -rotate.getRawAxis(Joystick.AxisType.kX.value),
                () -> robotCentric.getAsBoolean()
            )
        );
    }

    public void cancelDefaultTeleop() {
        s_Swerve.getDefaultCommand().cancel();
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
    }

    public SequentialCommandGroup MountAndBalance(Swerve s_Swerve){
        // get on the charge station, then balance, then put the wheels in an x configuration
        return new SequentialCommandGroup(
            new MountChargeStation(s_Swerve, false),
            new BalanceChargeStation(s_Swerve).andThen(new InstantCommand(() -> s_Swerve.configToX()))
        );
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return MountAndBalance(s_Swerve);
    }
}
