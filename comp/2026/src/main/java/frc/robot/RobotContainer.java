// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

// import frc.robot.subsystems.roller.RollerSubsystem;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
// import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.RollerWrist;
import frc.robot.commands.WristRoller;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmIO;
import frc.robot.subsystems.arm.ArmIOReal;
import frc.robot.subsystems.arm.ArmIOSim;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.ElevatorIOReal;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.subsystems.elevator.ElevatorState;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOReal;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.wrist.Wrist;
import frc.robot.subsystems.wrist.WristIO;
import frc.robot.subsystems.wrist.WristIOReal;
import frc.robot.subsystems.wrist.WristIOSim;

import static edu.wpi.first.units.Units.Rotations;

import org.bobcatrobotics.Controllers.ControllerAutoDetect;
import org.bobcatrobotics.Controllers.Gamepads.ControllerBase;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
        // Subsystems
        private final Drive drive;
        private Vision vision;
        private Intake intake;
        private Elevator elevator;
        private Wrist wrist;
        private Arm arm;

        // Controller
        private final ControllerBase controller;
        private final ControllerBase operator;

        double middlePosition = ElevatorState.MIDDLE.position.in(Rotations);
        // Dashboard inputs
        private final LoggedDashboardChooser<Command> autoChooser;

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {
                controller = ControllerAutoDetect.createGamepad(0, "driver");
                operator = ControllerAutoDetect.createGamepad(1, "operator");
                switch (Constants.currentMode) {
                        case REAL:
                                // Real robot, instantiate hardware IO implementations
                                drive = new Drive(new GyroIOPigeon2(),
                                                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                                                new ModuleIOTalonFX(TunerConstants.FrontRight),
                                                new ModuleIOTalonFX(TunerConstants.BackLeft),
                                                new ModuleIOTalonFX(TunerConstants.BackRight));
                                // Vision
                                vision = new Vision(drive::addVisionMeasurement,
                                                new VisionIOLimelight("", drive::getRotation));

                                intake = new Intake(new IntakeIOReal());
                                arm = new Arm(new ArmIOReal());
                                elevator = new Elevator(new ElevatorIOReal());
                                wrist = new Wrist(new WristIOReal());
                                break;
                        case SIM:
                                // Sim robot, instantiate physics sim IO implementations
                                drive = new Drive(new GyroIO() {},
                                                new ModuleIOSim(TunerConstants.FrontLeft),
                                                new ModuleIOSim(TunerConstants.FrontRight),
                                                new ModuleIOSim(TunerConstants.BackLeft),
                                                new ModuleIOSim(TunerConstants.BackRight));
                                intake = new Intake(new IntakeIOSim());
                                arm = new Arm(new ArmIOSim());
                                elevator = new Elevator(new ElevatorIOSim());
                                wrist = new Wrist(new WristIOSim());
                                break;

                        default:
                                // Replayed robot, disable IO implementations
                                drive = new Drive(new GyroIO() {}, new ModuleIO() {},
                                                new ModuleIO() {}, new ModuleIO() {},
                                                new ModuleIO() {});
                                intake = new Intake(new IntakeIO() {});
                                arm = new Arm(new ArmIO() {});
                                elevator = new Elevator(new ElevatorIO() {});
                                wrist = new Wrist(new WristIO() {});
                                break;
                }

                // Set up auto routines
                autoChooser = new LoggedDashboardChooser<>("Auto Choices",
                                AutoBuilder.buildAutoChooser());

                // Set up SysId routines
                autoChooser.addOption("Drive Wheel Radius Characterization",
                                DriveCommands.wheelRadiusCharacterization(drive));
                autoChooser.addOption("Drive Simple FF Characterization",
                                DriveCommands.feedforwardCharacterization(drive));
                autoChooser.addOption("Drive SysId (Quasistatic Forward)",
                                drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
                autoChooser.addOption("Drive SysId (Quasistatic Reverse)",
                                drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
                autoChooser.addOption("Drive SysId (Dynamic Forward)",
                                drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
                autoChooser.addOption("Drive SysId (Dynamic Reverse)",
                                drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

                // Configure the button bindings
                configureButtonBindings();
        }

        /**
         * Use this method to define your button->command mappings. Buttons can be created by
         * instantiating a {@link GenericHID} or one of its subclasses
         * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it
         * to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
         */
        private void configureButtonBindings() {
                // Default command, normal field-relative drive
                drive.setDefaultCommand(DriveCommands.joystickDrive(drive,
                                () -> -controller.getLeftY(), () -> -controller.getLeftX(),
                                () -> -controller.getRightX()));

                // Lock to 0° when A button is held
                controller.getButton("A").whileTrue(DriveCommands.joystickDriveAtAngle(drive,
                                () -> -controller.getLeftY(), () -> -controller.getLeftX(),
                                () -> new Rotation2d()));

                // Switch to X pattern when X button is pressed
                controller.getButton("X").onTrue(Commands.runOnce(drive::stopWithX, drive));

                // Reset gyro to 0° when B button is pressed
                controller.getButton("B").onTrue(Commands.runOnce(() -> drive.setPose(
                                new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                                drive).ignoringDisable(true));

                //operator.getButton("X").whileTrue(new RunCommand(() -> intake.runIntakeOut()))
                                //.onFalse(new InstantCommand(() -> intake.stop()));

                //operator.getButton("Y").whileTrue(new RunCommand(() -> intake.runIntakeOut()))
                                //.onFalse(new InstantCommand(() -> intake.stop()));

                operator.getButton("A")
                                .whileTrue(new RunCommand(() -> wrist.getOutofTheWay(), wrist).andThen(new RunCommand(() -> elevator.elevate(.5), elevator)))
                                .onFalse(new RunCommand(() -> elevator.holdPosition()));

                operator.getButton("B")
                                .whileTrue(new RunCommand(() -> wrist.getOutofTheWay(), wrist).andThen(new RunCommand(() -> elevator.elevate(-.5), elevator)))
                                .onFalse(new RunCommand(() -> elevator.holdPosition(), elevator));
                operator.getRightTrigger()
                                .whileTrue(new RunCommand(() -> wrist.setSpeed(1), wrist))
                                .onFalse(new InstantCommand(() -> wrist.stop()));
                operator.getLeftTrigger()
                                .whileTrue(new RunCommand(() -> wrist.setSpeed(-1), wrist))
                                .onFalse(new InstantCommand(() -> wrist.stop()));
                operator.getPovUp()
                                .whileTrue(new RunCommand(() -> arm.setPercent(1), wrist))
                                .onFalse(new InstantCommand(() -> arm.stop()));
                operator.getPovDown()
                                .whileTrue(new RunCommand(() -> arm.setPercent(-1), wrist))
                                .onFalse(new InstantCommand(() -> arm.stop()));
                //operator.getButton("X")
                                //.whileTrue(new RunCommand(() -> elevator.setState(middlePosition), elevator));
                operator.getButton("Y")
                                .whileTrue(new RollerWrist(wrist, intake))
                                .onFalse(new InstantCommand(() -> wrist.setSpeed(0)).alongWith(new InstantCommand(() -> intake.stop())));
                operator.getButton("X")
                                .whileTrue(new WristRoller(wrist, intake))
                                .onFalse(new InstantCommand(() -> wrist.setSpeed(0)).alongWith(new InstantCommand(() -> intake.stop())));

                

                

                

                


        }



        /**
         * Use this to pass the autonomous command to the main {@link Robot} class.
         *
         * @return the command to run in autonomous
         */
        public Command getAutonomousCommand() {
                return autoChooser.get();
        }

        public Pose2d getPose2D() {
                return drive.getPose();
        }

        public ControllerBase getControllers() {
                return controller;
        }
}