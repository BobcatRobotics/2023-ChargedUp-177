package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class Elevator extends SubsystemBase {
  private TalonFX elevatorMotor;

  private DigitalInput topLimit;
  private DigitalInput bottomLimit;
  
final MotionMagicVoltage m_motmag = new MotionMagicVoltage(0);
  private double holdPosValue;

  /** Creates a new Elevator. */
  public Elevator() {
    elevatorMotor = new TalonFX(ElevatorConstants.elevatorMotorPort);
    topLimit = new DigitalInput(ElevatorConstants.topLimitPort);
    bottomLimit = new DigitalInput(ElevatorConstants.bottomLimitPort);

    TalonFXConfiguration internalConfig =  new TalonFXConfiguration();
    internalConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    Slot0Configs slot0Configs = new Slot0Configs();
    slot0Configs.kP =  0.25;
    slot0Configs.kI =  0;
    slot0Configs.kD =  0;
    internalConfig.withSlot0(slot0Configs);
    internalConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    internalConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    internalConfig.MotionMagic.MotionMagicCruiseVelocity = 8000;
    internalConfig.MotionMagic.MotionMagicAcceleration = 4000;
    elevatorMotor.getConfigurator().apply(internalConfig);
    holdPosValue = elevatorMotor.getPosition().getValueAsDouble();
  }

  public void elevate(double speed) {
    elevatorMotor.set(speed);
  }
  public double getSpeed(){
    return elevatorMotor.getVelocity().getValueAsDouble();
  } 
  public boolean getStopped(){
    return getSpeed() == 0;
  }
  public void holdPosition() {
    elevatorMotor.setControl(m_motmag.withPosition(holdPosValue));
  }

  public void holdPosition(double pos) {
    elevatorMotor.setControl(m_motmag.withPosition(pos));
  }

  public double getPIDError(){
    return elevatorMotor.getClosedLoopError().getValueAsDouble();
  }

  public void setHoldPos() {
    holdPosValue = elevatorMotor.getPosition().getValueAsDouble();
  }

  public void resetEncoderPos() {
    elevatorMotor.getPosition().getValueAsDouble();
  }

  public boolean isAtSetpoint() {
    double error = elevatorMotor.getClosedLoopError().getValueAsDouble();
    double threshold = 0.5;
    if( error <= threshold){
      return true;
    }
    return false;
  }

  public boolean getBottomLimits() {
    return !bottomLimit.get();
  }
  public double getEncoderPos() {
    return elevatorMotor.getPosition().getValueAsDouble();
  }

  public boolean topLimitSwitch() {
    return !topLimit.get();
  }

  public void setState(int state) {
    if (state == 0) {
      elevatorMotor.setControl(m_motmag.withPosition(ElevatorConstants.pos0));
      holdPosValue = ElevatorConstants.pos0;
      holdPosition();
      SmartDashboard.putString("elevator error", "State: " + state + ", Error: " + getPIDError());
    } else if (state == 1) {
      elevatorMotor.setControl(m_motmag.withPosition(ElevatorConstants.pos1));
      holdPosValue = ElevatorConstants.pos1;
      holdPosition();
      SmartDashboard.putString("elevator error", "State: " + state + ", Error: " + getPIDError());
    } else if (state == 2) {
      elevatorMotor.setControl(m_motmag.withPosition(ElevatorConstants.pos2));
      holdPosValue = ElevatorConstants.pos2;
      holdPosition();
      SmartDashboard.putString("elevator error", "State: " + state + ", Error: " + getPIDError());
    }
  }

  public int getState() {
    double pos = elevatorMotor.getPosition().getValueAsDouble();
    if (pos <= 256) pos = 0;
    return (int) Math.ceil(pos/4096);
  }

  public double getEncoder() {
    return elevatorMotor.getPosition().getValueAsDouble();
  }

  public boolean isAtCurrentLimit() {
    double statorCurrent = elevatorMotor.getStatorCurrent().getValueAsDouble();
    return statorCurrent >= 50.0;
  }

  public void resetEncoderPosTop() {
    elevatorMotor.setPosition(-236710);
  }

  @Override
  public void periodic() {
    if (getBottomLimits()){
      resetEncoderPos();
      holdPosValue = 0;
      elevatorMotor.set( 0);
    }
  }
}