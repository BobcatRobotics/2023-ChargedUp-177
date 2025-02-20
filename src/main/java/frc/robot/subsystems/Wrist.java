// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Wrist extends SubsystemBase {
  /** Creates a new Wrist. */
  // TalonFX motor;
  Solenoid solenoid;
  double pressure;
  PneumaticHub phub;
  Compressor compressor;

  public Wrist() {
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
  public boolean getWristSolenoid(){
    return solenoid.get();
  }
  @Override
  public void periodic() {
    compressor.enableAnalog(80, 115);//TODO: check limits
    SmartDashboard.putNumber("compressor psi", compressor.getPressure());
  }
}
