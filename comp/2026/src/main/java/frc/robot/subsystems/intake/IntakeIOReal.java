package frc.robot.subsystems.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.signals.*;
import frc.robot.Constants;

public class IntakeIOReal implements IntakeIO {

    private final TalonFX motor;
    private final DutyCycleOut duty = new DutyCycleOut(0);

    private final StatusSignal<?> statorCurrent;
    private final StatusSignal<?> velocityRps;
    private final StatusSignal<?> dutyCycle;

    public IntakeIOReal() {
        motor = new TalonFX(Constants.intakeMotorID);

        var config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        motor.getConfigurator().apply(config);

        statorCurrent = motor.getStatorCurrent();
        velocityRps = motor.getVelocity();
        dutyCycle = motor.getDutyCycle();
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        BaseStatusSignal.refreshAll(statorCurrent, velocityRps, dutyCycle);

        inputs.statorCurrent = statorCurrent.getValueAsDouble();
        inputs.velocityRps = velocityRps.getValueAsDouble();
        inputs.percentOutput = dutyCycle.getValueAsDouble();
    }

    @Override
    public void setPercent(double percent) {
        motor.setControl(duty.withOutput(percent));
    }
}