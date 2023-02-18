// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Util.MathUtils;

public class Wrist extends SubsystemBase {
  /** Creates a new Wrist. */
  TalonFX motor;
  Solenoid solenoid;
  double pressure;
  PneumaticHub phub;
  Compressor compressor;

  public Wrist() {
    motor  = new TalonFX(Constants.intakeMotorID);
    solenoid = new Solenoid(PneumaticsModuleType.REVPH, Constants.wristSolenoidID);
    phub = new PneumaticHub(Constants.pHubID);
    compressor = new Compressor(Constants.compressorID, PneumaticsModuleType.REVPH);
    
  }
  public void wristSolenoidON(){
    solenoid.set(true);
  }
  public void wristSolenoidOFF(){
    solenoid.set(false);
  }

  // public void turnWrist(double speed){
  //   speed = MathUtils.throttlePercent(speed);
  //   //if lower limit switch is tripped and we're trying to go down, don't
  //   //if upper limit switch is tripped and we're trying to go up, don't
  //   //otherwise drive at given speed
  //   if(!((Math.signum(speed) == -1 && lowerLimit.get() == true))){
  //     if(!((Math.signum(speed) == 1 && lowerLimit.get() == false))){
  //       motor.set(TalonFXControlMode.PercentOutput, speed);
  //     }
  //   }
  // }
  @Override
  public void periodic() {
    compressor.enableAnalog(80, 115);//TODO: check limits
    SmartDashboard.putNumber("compressor psi", compressor.getPressure());
  }
}
