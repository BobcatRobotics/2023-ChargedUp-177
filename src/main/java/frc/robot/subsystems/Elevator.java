package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;

public class Elevator extends SubsystemBase {
  private WPI_TalonFX elevatorMotor;

  private DigitalInput topLimit;
  private DigitalInput bottomLimit;
  

  private double holdPosValue;

  /** Creates a new Elevator. */
  public Elevator() {
    elevatorMotor = new WPI_TalonFX(ElevatorConstants.elevatorMotorPort);
    topLimit = new DigitalInput(ElevatorConstants.topLimitPort);
    bottomLimit = new DigitalInput(ElevatorConstants.bottomLimitPort);

    elevatorMotor.configFactoryDefault();
    elevatorMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 20);
    elevatorMotor.setSensorPhase(true);
    elevatorMotor.configNominalOutputForward(0, 20);
    elevatorMotor.configNominalOutputReverse(0, 20);
    elevatorMotor.configPeakOutputForward(0.75, 20);
    elevatorMotor.configPeakOutputReverse(-0.75, 20);//TODO: needs to be changes for comp
    elevatorMotor.configAllowableClosedloopError(0, 0, 20);
    elevatorMotor.config_kF(0, 0, 20);
    elevatorMotor.config_kP(0, 0.25, 20);
    elevatorMotor.config_kI(0, 0, 20);
    elevatorMotor.config_kD(0, 0, 20);
    elevatorMotor.setInverted(false); // used to be true but we flipped motor direction 2/25/23
    elevatorMotor.setNeutralMode(NeutralMode.Brake);
    elevatorMotor.configNeutralDeadband(0.001, 20);
    elevatorMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, 20);
    elevatorMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, 20);
    elevatorMotor.configAllowableClosedloopError(0, 400, 20);


    // motion magic trapezoid configuration
    //elevatorMotor.configAllowableClosedloopError()
    elevatorMotor.configMotionCruiseVelocity(2000, 20); //needs to be tuned to robot
    elevatorMotor.configMotionAcceleration(1000, 20);

    holdPosValue = elevatorMotor.getSelectedSensorPosition();
  }

  public void elevate(double speed) {
    elevatorMotor.set(ControlMode.PercentOutput, speed);
  }
  public double getSpeed(){
    return elevatorMotor.getSelectedSensorVelocity();
  } 
  public boolean getStopped(){
    return getSpeed() == 0;
  }
  public void holdPosition() {
    elevatorMotor.set(ControlMode.Position, holdPosValue);
  }

  public void holdPosition(double pos) {
    elevatorMotor.set(ControlMode.Position, pos);
  }

  public double getPIDError(){
    return elevatorMotor.getClosedLoopError();
  }

  public void setHoldPos() {
    holdPosValue = elevatorMotor.getSelectedSensorPosition();
  }

  public void resetEncoderPos() {
    elevatorMotor.setSelectedSensorPosition(0);
  }

  public boolean isAtSetpoint() {
    return elevatorMotor.isMotionProfileFinished();
  }

  // public boolean getTopLimits() {
  //   return !topLimit.get();
  // }

  public boolean getBottomLimits() {
    return !bottomLimit.get();
  }
  public double getEncoderPos() {
    return elevatorMotor.getSelectedSensorPosition();
  }

  public boolean topLimitSwitch() {
    return !topLimit.get();
  }

  public void setState(int state) {
    if (state == 0) {
      elevatorMotor.set(ControlMode.MotionMagic, ElevatorConstants.pos0);
      holdPosValue = ElevatorConstants.pos0;
      holdPosition();
      SmartDashboard.putString("elevator error", "State: " + state + ", Error: " + getPIDError());
    } else if (state == 1) {
      elevatorMotor.set(ControlMode.MotionMagic, ElevatorConstants.pos1);
      holdPosValue = ElevatorConstants.pos1;
      holdPosition();
      SmartDashboard.putString("elevator error", "State: " + state + ", Error: " + getPIDError());
    } else if (state == 2) {
      elevatorMotor.set(ControlMode.MotionMagic, ElevatorConstants.pos2);
      holdPosValue = ElevatorConstants.pos2;
      holdPosition();
      SmartDashboard.putString("elevator error", "State: " + state + ", Error: " + getPIDError());
    }
  }

  public int getState() {
    double pos = elevatorMotor.getSelectedSensorPosition();
    if (pos <= 256) pos = 0;
    return (int) Math.ceil(pos/4096);
  }

  public double getEncoder() {
    return elevatorMotor.getSelectedSensorPosition();
  }

  public boolean isAtCurrentLimit() {
    return elevatorMotor.getStatorCurrent() >= 35.0;
  }

  public void resetEncoderPosTop() {
    elevatorMotor.setSelectedSensorPosition(-236710);
  }

  @Override
  public void periodic() {
    if (getBottomLimits()){
      resetEncoderPos();
      holdPosValue = 0;
      elevatorMotor.set(ControlMode.PercentOutput, 0);
    }
    // This method will be called once per scheduler run
  }
}