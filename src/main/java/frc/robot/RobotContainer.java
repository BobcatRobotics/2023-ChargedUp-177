package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

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
    private final Joystick driver = new Joystick(2);
    private final Joystick rotate = new Joystick(0);
    private final Joystick strafe = new Joystick(1);

    // /* Drive Controls */
    // private final int translationAxis = XboxController.Axis.kLeftY.value;
    // private final int strafeAxis = XboxController.Axis.kLeftX.value;
    // private final int rotationAxis = XboxController.Axis.kRightX.value;




    // Y -> zero gyro
    // LB -> robot centric
    // A -> start music

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
    private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    private final JoystickButton startMusic = new JoystickButton(driver, XboxController.Button.kA.value);

    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();

    /*do we want to play music or drive*/
    private boolean playMusic = true;


    //TODO: implement sendable chooser
    private String song = "hbfs";
    private String path;
    

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        switch (song) {
            case "hbfs":
                path = "hbfs.chrp";
                break;
            case "DreamOn":
                path = "DreamOn.chrp";
                break;
            case "Megalovania":
                path = "Megalovania.chrp";
                break;
            case "goodRiddance":
                path = "goodRiddance.chrp";
                break;
            case "justLikeThis":
                path = "justLikeThis.chrp";
                break;
            case "sandstorm":
                path = "sandstorm.chrp";
                break;
            default:
                path = "hbfs.chrp";
                break;
        }

        if(playMusic){
            s_Swerve.setDefaultCommand(
                new Music(
                    s_Swerve, 
                    path,
                    () -> playMusic
                )
            );
        }
        else{
        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> -strafe.getRawAxis(Joystick.AxisType.kY.value), 
                () -> -strafe.getRawAxis(Joystick.AxisType.kX.value), 
                () -> -rotate.getRawAxis(Joystick.AxisType.kX.value), 
                () -> robotCentric.getAsBoolean()
            )
        );}

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
        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return new exampleAuto(s_Swerve);
    }
}
