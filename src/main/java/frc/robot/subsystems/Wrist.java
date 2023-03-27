// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

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
  
  WPI_TalonFX motor;
  CANCoder encoder;

  public Wrist() {
    motor  = new WPI_TalonFX(Constants.wristMotorID);
    encoder = new CANCoder(Constants.wristCANCoderID);

    
    
    motor.configFactoryDefault();
    motor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 20);
    motor.setSensorPhase(true);
    motor.configNominalOutputForward(0, 20);
    motor.configNominalOutputReverse(0, 20);
    motor.configPeakOutputForward(0.5, 20);
    motor.configPeakOutputReverse(-0.5, 20);
    motor.configAllowableClosedloopError(0, 0, 20);
    motor.config_kF(0, 0, 20);
    motor.config_kP(0, 0.275, 20); //TODO: Tune!
    motor.config_kI(0, 0, 20);
    motor.config_kD(0, 0, 20);
    motor.setInverted(true);
    motor.setNeutralMode(NeutralMode.Brake);
    motor.configNeutralDeadband(0.001, 20);
    motor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, 20);
    motor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, 20);
    motor.configMotionCruiseVelocity(30000, 20); //needs to be tuned to robot
    motor.configMotionAcceleration(24000, 20);
    motor.configAllowableClosedloopError(0, 200, 20);

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

  /**
   * returns degrees!
   */
  public double getWristEncoderPos(){
    SmartDashboard.putNumber("wrist encoder", encoder.getAbsolutePosition());
    return encoder.getAbsolutePosition(); //TODO: check this!
  }
  public void setSpeed(double speed){
    motor.set(ControlMode.PercentOutput, speed);
  }
  public void stop() {
    motor.set(0);
  }
  public void setState(WristState state){
    motor.set(ControlMode.MotionMagic, stateToEncoderCount(state));
  }

  public int stateToEncoderCount(WristState state){
    switch(state){
      case forwardGround:
        return Constants.WristConstants.forwardGround;
      case topGround:
        return Constants.WristConstants.topGround;
      case HPChute:
        return Constants.WristConstants.HPChute;
      case HPSlide:
        return Constants.WristConstants.HPSlide;
      case score:
        return Constants.WristConstants.score;
      default:
        return 0;   
  }
}
  public boolean topLimit(){
    return getWristEncoderPos() > Constants.WristConstants.topLimit;
  }

  @Override
  public void periodic() {

  }
}
