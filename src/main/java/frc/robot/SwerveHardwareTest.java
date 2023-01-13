// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;



import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import edu.wpi.first.wpilibj2.command.CommandBase;


  //  quick and dirty hardware test, not the best code
  //  so dont use this command as a 
  //  reference/learning tool cause its bad 


  // MAKE SURE THE ROBOTS WHEELS ARE NOT TOUCHING THE
  // GROUND WHEN YOU RUN THIS COMMAND
  
  //TODO: add documentation
public class SwerveHardwareTest extends CommandBase {
  /** Creates a new SwerveHardwareTest. */

  int lbaID = 6; //TODO: can IDs?
  int lbdID = 5; //I took these IDs from the swerve testing branch
  int rbaID = 8; //idk if they're right tho
  int rbdID = 7;
  int lfaID = 2;
  int lfdID = 1;
  int rfaID = 4;
  int rfdID = 3;
  int lbCAN = 3;
  int rbCAN = 4;
  int rfCAN = 2;
  int lfCAN = 1;

  // back left module
  TalonFX lbAngleMotor = new TalonFX(lbaID);
  TalonFX lbDriveMotor = new TalonFX(lbdID);
  CANCoder lbEncoder = new CANCoder(lbCAN);

  // back right module
  TalonFX rbAngleMotor = new TalonFX(rbaID);
  TalonFX rbDriveMotor = new TalonFX(rbdID);
  CANCoder rbEncoder = new CANCoder(rbCAN);

  // front left module
  TalonFX lfAngleMotor = new TalonFX(lfaID);
  TalonFX lfDriveMotor = new TalonFX(lfdID);
  CANCoder lfEncoder = new CANCoder(lfCAN);

  // front right module
  TalonFX rfAngleMotor = new TalonFX(rfaID);
  TalonFX rfDriveMotor = new TalonFX(rfdID);
  CANCoder rfEncoder = new CANCoder(rfCAN);

  //encoder counts per rotation
  int falconCpr = 2048;
  int canCoderCPR = 4069;

  //gear ratio for the drive motors
  double gearRatio = 6.12;

  //the cpr of one full rotation of the drive motors
  double cPerRotation = falconCpr * gearRatio;

  //is the command finished
  private boolean fin = false;

  //do we want the cool test or the simple test
  private boolean fancy = true;

  public SwerveHardwareTest() {
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  //reset all encoders
  lbDriveMotor.setSelectedSensorPosition(0);
  rbDriveMotor.setSelectedSensorPosition(0);
  lfDriveMotor.setSelectedSensorPosition(0);
  rfDriveMotor.setSelectedSensorPosition(0);
  lbEncoder.setPosition(0);
  rbEncoder.setPosition(0);
  lfEncoder.setPosition(0);
  rfEncoder.setPosition(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    //if we want the cool test
    if (fancy){

    //start rotating the left back angle motor
    lbAngleMotor.set(ControlMode.PercentOutput, 0.5);

    //once it makes a full rotation, start rotating the lb drive motor
    if (lbEncoder.getPosition() > canCoderCPR) {
      lbDriveMotor.set(ControlMode.PercentOutput, 0.5);  
    }
    //once the lb drive motor makes a full rotation, start rotating the right back angle motor
    if (lbDriveMotor.getSelectedSensorPosition() > cPerRotation) {
      rbAngleMotor.set(ControlMode.PercentOutput, 0.5);
    }
    //repeat for all motors
    if (rbEncoder.getPosition() > canCoderCPR) {
      rbDriveMotor.set(ControlMode.PercentOutput, 0.5);
    }if(rbDriveMotor.getSelectedSensorPosition() > cPerRotation) {
      lfAngleMotor.set(ControlMode.PercentOutput, 0.5);
    }

    if (lfEncoder.getPosition() > canCoderCPR){
      lfDriveMotor.set(ControlMode.PercentOutput, 0.5);
    }if(lfDriveMotor.getSelectedSensorPosition() > cPerRotation) {
      rfAngleMotor.set(ControlMode.PercentOutput, 0.5);
    }
    
    if (rfEncoder.getPosition() > canCoderCPR) {
      rfDriveMotor.set(ControlMode.PercentOutput, 0.5);
    }

    //once all motors have made a full rotation, run all motors at 100% for 3 seconds
    if(rfDriveMotor.getSelectedSensorPosition() > cPerRotation){
    lbAngleMotor.set(ControlMode.PercentOutput, 1);
    lbDriveMotor.set(ControlMode.PercentOutput, 1);
    rbAngleMotor.set(ControlMode.PercentOutput, 1);
    rbDriveMotor.set(ControlMode.PercentOutput, 1);
    lfAngleMotor.set(ControlMode.PercentOutput, 1);
    lfDriveMotor.set(ControlMode.PercentOutput, 1);
    rfAngleMotor.set(ControlMode.PercentOutput, 1);
    rfDriveMotor.set(ControlMode.PercentOutput, 1);

    //wait 3 seconds
    edu.wpi.first.wpilibj.Timer.delay(3);
    }

    //then stop all motors
    lbAngleMotor.set(ControlMode.PercentOutput, 0);
    lbDriveMotor.set(ControlMode.PercentOutput, 0);
    rbAngleMotor.set(ControlMode.PercentOutput, 0);
    rbDriveMotor.set(ControlMode.PercentOutput, 0);
    lfAngleMotor.set(ControlMode.PercentOutput, 0);
    lfDriveMotor.set(ControlMode.PercentOutput, 0);
    rfAngleMotor.set(ControlMode.PercentOutput, 0);
    rfAngleMotor.set(ControlMode.PercentOutput, 0);
    
    //this makes it so that isFinished() returns true, which ends the command
    fin = true;
  }
  else{
    //if we want the simple test
    //run all motors at 100% for 3 seconds
    //then stop all motors
    lbAngleMotor.set(ControlMode.PercentOutput, 1);
    lbDriveMotor.set(ControlMode.PercentOutput, 1);
    rbAngleMotor.set(ControlMode.PercentOutput, 1);
    rbDriveMotor.set(ControlMode.PercentOutput, 1);
    lfAngleMotor.set(ControlMode.PercentOutput, 1);
    lfDriveMotor.set(ControlMode.PercentOutput, 1);
    rfAngleMotor.set(ControlMode.PercentOutput, 1);
    rfDriveMotor.set(ControlMode.PercentOutput, 1);

    edu.wpi.first.wpilibj.Timer.delay(3);

    lbAngleMotor.set(ControlMode.PercentOutput, 0);
    lbDriveMotor.set(ControlMode.PercentOutput, 0);
    rbAngleMotor.set(ControlMode.PercentOutput, 0);
    rbDriveMotor.set(ControlMode.PercentOutput, 0);
    lfAngleMotor.set(ControlMode.PercentOutput, 0);
    lfDriveMotor.set(ControlMode.PercentOutput, 0);
    rfAngleMotor.set(ControlMode.PercentOutput, 0);
    rfDriveMotor.set(ControlMode.PercentOutput, 0);

    fin = true;
  }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return fin;
  }
}
