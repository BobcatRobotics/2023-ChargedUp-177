// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Util.MathUtils;

public class Intake extends SubsystemBase {
  private TalonFX motor; 

  /** Creates a new Intake. */
  public Intake() {
   motor = new TalonFX(Constants.intakeMotorID);
   TalonFXConfiguration internalConfig = new TalonFXConfiguration();
   internalConfig.MotorOutput.withNeutralMode(NeutralModeValue.Brake);
   motor.getConfigurator().apply(internalConfig);
  }
  
  public void runIntakeIn(){
    motor.set(-0.9);
  }
  public void runIntakeOut(){
    motor.set(0.4);
  } 
  public void runIntakeOutFull(){
    motor.set(1);
  }

  public boolean isAtHardStop() {
    double statorCurrent = motor.getStatorCurrent().getValueAsDouble();
    return statorCurrent >= 20.0;
  }

  public void runIntakePercent(double speed){
    speed = MathUtils.throttlePercent(speed);
    motor.set(speed);
  }
  public void stop(){
    motor.stopMotor();
  }
  @Override
  public void periodic() {
    
  }
}
