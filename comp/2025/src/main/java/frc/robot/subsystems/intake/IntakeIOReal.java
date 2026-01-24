package frc.robot.subsystems.intake;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.signals.*;
import frc.robot.Constants;

public class IntakeIOReal implements IntakeIO {

    private final TalonFX motor;
    private final DutyCycleOut duty = new DutyCycleOut(0);

    public IntakeIOReal() {
        motor = new TalonFX(Constants.intakeMotorID);

        var config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        motor.getConfigurator().apply(config);
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.statorCurrent = motor.getStatorCurrent().getValueAsDouble();
        inputs.velocityRps = motor.getVelocity().getValueAsDouble();
        inputs.percentOutput = motor.getDutyCycle().getValueAsDouble();
    }

    @Override
    public void setPercent(double percent) {
        motor.setControl(duty.withOutput(percent));
    }
}
