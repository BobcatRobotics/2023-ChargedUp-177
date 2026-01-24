package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.hardware.TalonFX;

import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.signals.*;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorIOReal implements ElevatorIO {

    private final TalonFX motor;
    private final DigitalInput topLimit;
    private final DigitalInput bottomLimit;

    private final MotionMagicVoltage mmRequest = new MotionMagicVoltage(0);
    private final DutyCycleOut dutyRequest = new DutyCycleOut(0);

    public ElevatorIOReal() {
        motor = new TalonFX(ElevatorConstants.elevatorMotorPort);
        topLimit = new DigitalInput(ElevatorConstants.topLimitPort);
        bottomLimit = new DigitalInput(ElevatorConstants.bottomLimitPort);

        var config = new TalonFXConfiguration();

        // PID slot 0
        config.Slot0.kP = 0.25;
        config.Slot0.kI = 0.0;
        config.Slot0.kD = 0.0;
        config.Slot0.kV = 0.0;

        // Output
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        // MotionMagic
        config.MotionMagic.MotionMagicCruiseVelocity = 8000;
        config.MotionMagic.MotionMagicAcceleration = 4000;

        // Limits
        ElevatorState max_limit = ElevatorState.TOP;
        ElevatorState min_limit = ElevatorState.BOTTOM;
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 0;
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = max_limit.position.in(Rotations);

        motor.getConfigurator().apply(config);
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        inputs.positionRotations = motor.getPosition().getValueAsDouble();
        inputs.velocityRps = motor.getVelocity().getValueAsDouble();
        inputs.statorCurrent = motor.getStatorCurrent().getValueAsDouble();
        inputs.bottomLimitPressed = !bottomLimit.get();
        inputs.topLimitPressed = !topLimit.get();
        inputs.isMotionDone = motor.getClosedLoopError().getValueAsDouble() < 400;
    }

    @Override
    public void setPercent(double percent) {
        motor.setControl(dutyRequest.withOutput(percent));
    }

    @Override
    public void setMotionMagic(double position) {
        motor.setControl(mmRequest.withPosition(position));
    }

    @Override
    public void holdPosition(double position) {
        motor.setControl(mmRequest.withPosition(position));
    }

    @Override
    public void resetEncoder() {
        motor.setPosition(0);
    }

    @Override
    public void resetEncoderTop() {
        motor.setPosition(-236710);
    }
}
