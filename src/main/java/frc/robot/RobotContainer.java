package frc.robot;



import java.util.List;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
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
import frc.robot.commands.Autos.AlignToTargetAutos;
import frc.robot.commands.Autos.BalanceChargeStation;
import frc.robot.commands.Autos.MountAndBalance;
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
import frc.robot.commands.IntakeOut;

import frc.robot.autos.PathPlannerTest;
//import frc.robot.autos.RedHighCone6PickupBalance;
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
    //private Constants.ButtonHashtable bh = new Constants.ButtonHashtable();

    
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
    //private final JoystickButton alignRobot = new JoystickButton(driver, 2);
    //private final JoystickButton zeroGyro = new JoystickButton(driver, 4);
    
    //ruffy buttons
    private final JoystickButton ruffy0 = new JoystickButton(rotate, 1);
    private final JoystickButton ruffy1 = new JoystickButton(strafe, 1);

   // private final JoystickButton alignRobot = new JoystickButton(driver, 1);


   private final JoystickButton leftBumper = new JoystickButton(driver, 5); //left bumper
   private final JoystickButton rightBumper = new JoystickButton(driver, 6);//right bumper
   private final JoystickButton a = new JoystickButton(driver, 2);
   private final JoystickButton b = new JoystickButton(driver, 3);
   private final JoystickButton righttrigger = new JoystickButton(driver, 8);
   private final JoystickButton lefttrigger = new JoystickButton(driver, 7);
   private final JoystickButton back = new JoystickButton(driver, 9);
   private final JoystickButton start = new JoystickButton(driver, 10);
   private final POVButton DUp = new POVButton(driver, 0);
   private final POVButton DLeft = new POVButton(driver, 270);
   private final POVButton DDown = new POVButton(driver, 180);
    /* Subsystems */


    public static Swerve s_Swerve = new Swerve();
    public static Limelight m_Limelight = new Limelight();

    private final Elevator m_Elevator = new Elevator();
    private final Intake m_Intake = new Intake();
    private final Arm m_Arm = new Arm();
    private final Wrist m_Wrist = new Wrist();
    
    /* Commands */
    private final Command elevatorControls = new ElevatorControls(m_Elevator, driver, m_Arm);
    private final Command armControls = new ArmControls(m_Arm, driver, m_Elevator);
    private final Command setarm0 = new SetArm(m_Arm,0);
    private final Command setarm1 = new SetArm(m_Arm,1);
    private final Command setarm2 = new SetArm(m_Arm,2);
    private final Command setelevator0 = new SetElevator(m_Elevator,0);
    private final Command setelevator2 = new SetElevator(m_Elevator,2);
    private final SequentialCommandGroup chargestation = new MountAndBalance(s_Swerve);
    
    
    


    //private final Command align = new AlignToTarget(s_Swerve, m_Limelight).withInterruptBehavior(InterruptionBehavior.kCancelIncoming).repeatedly();
    private final Command align = new AlignToTarget(s_Swerve, m_Limelight);
    //private final RedHighCone6PickupBalance redHighCone6PickupBalance = new RedHighCone6PickupBalance(s_Swerve, m_Limelight);
    private PathPlannerTest pathPlannerTest;
    private static SwerveAutoBuilder swerveAutoBuilder;

    /* SendableChooser */
    SendableChooser<SequentialCommandGroup> autoChooser = new SendableChooser<>();

    public void setUpAutos() {
        // Sendable Chooser Setup
        //autoChooser.setDefaultOption("Red High Cone 6 Pickup & Balance", redHighCone6PickupBalance);
        setUpEventMap();
        pathPlannerTest = new PathPlannerTest();
        autoChooser.setDefaultOption("PathPlanner Test", pathPlannerTest);
        //autoChooser.addOption("PathPlanner Test w/ Events", new SequentialCommandGroup(Swerve.followTrajectoryCommand(PathPlanner.loadPath("New Path", new PathConstraints(Constants.AutoConstants.kMaxSpeedMetersPerSecond, Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)), true)));
        //autoChooser.addOption("charge station", chargestation);
        SmartDashboard.putData(autoChooser);
    }

    public void setUpEventMap() {
        Constants.AutoConstants.eventMap.clear();
        Constants.AutoConstants.eventMap.put("chargeStation", new MountAndBalance(s_Swerve));
        Constants.AutoConstants.eventMap.put("align", new AlignToTargetAutos(s_Swerve, m_Limelight));
        Constants.AutoConstants.eventMap.put("highPreset", new ScoreMid(m_Elevator, m_Arm, m_Wrist));
        Constants.AutoConstants.eventMap.put("intakeGround", new ForwardSuck(m_Elevator, m_Arm, m_Wrist));
        Constants.AutoConstants.eventMap.put("startingConfig", new StartingConfig(m_Elevator, m_Arm, m_Wrist));
        Constants.AutoConstants.eventMap.put("flickWrist", new InstantCommand(m_Wrist::wristSolenoidON));
        Constants.AutoConstants.eventMap.put("intakeOut", new IntakeOut(m_Intake));//new ParallelRaceGroup(new IntakeOut(), new WaitCommand(5)));
    }

    public void printHashMap() {
        SmartDashboard.putString("eventMap", Constants.AutoConstants.eventMap.toString());
    }



    // returns the angle of the joystick in degrees
    public double getJoystickAngle(){
        return  ((Math.atan2(rotate.getRawAxis(Joystick.AxisType.kY.value), rotate.getRawAxis(Joystick.AxisType.kX.value)) * 180 / Math.PI)+360)%360;
    }

    public void displayGyro(){
        SmartDashboard.putNumber("pitch", s_Swerve.getPitch());
        SmartDashboard.putNumber("yaw", s_Swerve.getRoll());
    }

   
    public Command getDefaultCommand(){
        return s_Swerve.getDefaultCommand();
    }

    public void resetToAbsolute() {
        s_Swerve.resetModulesToAbsolute();
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
        setUpEventMap();
        configureButtonBindings();
    }


    public void scheduleDefaultTeleop() {
        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> -strafe.getRawAxis(Joystick.AxisType.kY.value)*Math.abs(strafe.getRawAxis(Joystick.AxisType.kY.value)), 
                () -> -strafe.getRawAxis(Joystick.AxisType.kX.value)*Math.abs(strafe.getRawAxis(Joystick.AxisType.kX.value)), 
                () -> -rotate.getRawAxis(Joystick.AxisType.kX.value), 
                () -> false //() -> robotCentric.getAsBoolean() //always field centric
            )
        );

        m_Arm.setDefaultCommand(armControls);
        m_Elevator.setDefaultCommand(elevatorControls);
        m_Intake.setDefaultCommand(new RunIntake(m_Intake, driver));


        // Configure the button bindings
        configureButtonBindings();
        m_Limelight.initializeLimeLight();
    }

    public void cancelDefaultTeleop() {
        s_Swerve.getDefaultCommand().cancel();
        m_Arm.getDefaultCommand().cancel();
        m_Elevator.getDefaultCommand().cancel();
        m_Intake.getDefaultCommand().cancel();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */

        //resetToAbsolute.onTrue(new InstantCommand(() -> s_Swerve.resetModulesToAbsolute()));
        //alignRobot.whileTrue(align);
        //zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));
        ruffy0.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));
        ruffy1.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));

        
        righttrigger.onTrue(new InstantCommand(m_Wrist::wristSolenoidOFF));
        rightBumper.onTrue(new InstantCommand(m_Wrist::wristSolenoidON));
        
        
    //    lefttrigger.whileTrue(new InstantCommand(m_Intake::runIntakeOut));
    //     leftBumper.whileTrue(new InstantCommand(m_Intake::runIntakeIn));

        // if(driver.getPOV() == 0){
        //     new ScoreHigh(m_Elevator, m_Arm, m_Intake, m_Wrist);
        // }
        DUp.onTrue(new ScoreHigh(m_Elevator, m_Arm, m_Intake, m_Wrist).until(this::anythingPressed));
        // if(driver.getPOV() == 180){
        //     new StartingConfig(m_Elevator, m_Arm);
        // }
        DDown.onTrue(new StartingConfig(m_Elevator, m_Arm, m_Wrist).until(this::anythingPressed));
        // if(driver.getPOV() == 270){
        //     new ScoreMid(m_Elevator, m_Arm, m_Intake, m_Wrist);
        // }
        DLeft.onTrue(new ScoreMid(m_Elevator, m_Arm, m_Wrist).until(this::anythingPressed));

        a.onTrue(new ForwardSuck(m_Elevator, m_Arm, m_Wrist).until(this::anythingPressed));
        b.onTrue(new TopSuck(m_Elevator, m_Arm, m_Intake, m_Wrist).until(this::anythingPressed));

        //back.whileTrue(new InstantCommand(m_Intake::runIntakeOutFull));

        //alignRobot.whileTrue(align);
        start.whileTrue(align);
    }

    public boolean anythingPressed() {
        return Math.abs(driver.getRawAxis(1)) >= 0.1 || Math.abs(driver.getRawAxis(3)) >= 0.1; 
    }



  
    
    public static Command buildAuto(List<PathPlannerTrajectory> trajs) {
        //s_Swerve.resetOdometry(trajs.get(0).getInitialHolonomicPose());
        swerveAutoBuilder = new SwerveAutoBuilder(
            s_Swerve::getPose,
            s_Swerve::resetOdometry,
            Constants.Swerve.swerveKinematics,
            new PIDConstants(Constants.AutoConstants.kPXController, 0, 0),
            new PIDConstants(Constants.AutoConstants.kPThetaController, 0, 0),
            s_Swerve::setModuleStates,
            Constants.AutoConstants.eventMap,
            true,
            s_Swerve
        );

        return swerveAutoBuilder.fullAuto(trajs);
    }
    




    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
       // s_Swerve.resetOdometry(new Pose2d(0, 0, s_Swerve.getYaw()));
       // return new MountAndBalance(s_Swerve); //autoChooser.getSelected();
       return autoChooser.getSelected();
    }
}
