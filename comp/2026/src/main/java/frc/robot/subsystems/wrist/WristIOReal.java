package frc.robot.subsystems.wrist;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;

import frc.robot.Constants;

public class WristIOReal implements WristIO {

  private final TalonFX motor = new TalonFX(Constants.wristMotorID);
  private final CANcoder encoder = new CANcoder(Constants.wristCANCoderID);

  private final DutyCycleOut percentReq = new DutyCycleOut(0);
  private final MotionMagicVoltage mmReq = new MotionMagicVoltage(0);

  public WristIOReal() {

    TalonFXConfiguration config = new TalonFXConfiguration();
    config.Slot0.kP = 0.275;
    config.Slot0.kI = 0;
    config.Slot0.kD = 0;
    config.Slot0.kV = 0;
    config.Slot0.kG = 0.3;
    config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    config.MotionMagic.MotionMagicCruiseVelocity = 30000;
    config.MotionMagic.MotionMagicAcceleration = 24000;

    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Constants.WristConstants.topLimit;
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Constants.WristConstants.bottomLimit;


    motor.getConfigurator().apply(config);

    encoder.getConfigurator().apply(new CANcoderConfiguration());
  }

  @Override
  public void updateInputs(WristIOInputs inputs) {
    inputs.absolutePositionDeg = encoder.getAbsolutePosition().getValueAsDouble();
    inputs.motorVelocity = motor.getVelocity().getValueAsDouble();
    inputs.motorOutput = motor.getDutyCycle().getValueAsDouble();
  }

  @Override
  public void setPercent(double percent) {
    percentReq.Output = percent;
    motor.setControl(percentReq);
  }

  @Override
  public void setMotionMagic(WristState state) {
    motor.setControl(mmReq.withPosition(state.position).withFeedForward(1));
  }

  @Override
  public void stop() {
    motor.setControl(new DutyCycleOut(0));
  }
}