// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
import java.io.FileNotFoundException;
import java.util.Scanner;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.Autos.BalanceChargeStation;
import frc.robot.subsystems.Swerve;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DataLogEntry;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  public static CTREConfigs ctreConfigs;

  private Command m_autonomousCommand;
  

  private RobotContainer m_robotContainer;
  
  private boolean firstExecute = true;

  private Timer timer;

  private int i = 0;

  DoubleLogEntry batteryVoltage;
  StringLogEntry encoderOdometry;
  StringLogEntry poseEstimatorOdometry;
  StringLogEntry limelightBotPose;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Starts recording to data log
    DataLogManager.start();

    DataLog log = DataLogManager.getLog();

    // Record both DS control and joystick data
    DriverStation.startDataLog(DataLogManager.getLog());
    //log.appendString(i, null, i);
    batteryVoltage = new DoubleLogEntry(log, "/my/batteryVoltage");
    encoderOdometry = new StringLogEntry(log, "/my/encoderOdometry");
    poseEstimatorOdometry = new StringLogEntry(log, "/my/poseEstimatorOdometry");
    limelightBotPose = new StringLogEntry(log, "/my/limelightBotPose");
    
    ctreConfigs = new CTREConfigs();
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    timer = new Timer();

    File deployDir = Filesystem.getDeployDirectory();
    File branchFile = new File(deployDir, "branch.txt");
    File commitFile = new File(deployDir, "commit.txt");
    String branch = "";
    String commit = "";

    try {
      Scanner scanner = new Scanner(branchFile);
      branch = scanner.nextLine();
      scanner.close();
      scanner = new Scanner(commitFile);
      commit = scanner.nextLine();
      scanner.close();
    } catch (FileNotFoundException e) {
      e.printStackTrace();
    }
    
    SmartDashboard.putString("Deployed code:", branch + " " + commit);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
  
    CommandScheduler.getInstance().cancelAll();
    
  }

  @Override
  public void disabledPeriodic() {
  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    RobotContainer.s_Swerve.resetOdometryAutos();
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    try{
    m_robotContainer.cancelDefaultTeleop();
    }catch(NullPointerException e){

    }

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    if(firstExecute){
      firstExecute =false;
      timer.start();
    }
    if (m_autonomousCommand.isFinished() && i==0) {
      timer.stop();
      i++;
    }
    SmartDashboard.putNumber("Time Path Ran For", timer.get());
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      System.out.println("Canceling auto command");
      m_autonomousCommand.cancel();
    }

    SmartDashboard.putNumber("PID Value", 0);
    m_robotContainer.resetToAbsolute();
    m_robotContainer.scheduleDefaultTeleop();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
