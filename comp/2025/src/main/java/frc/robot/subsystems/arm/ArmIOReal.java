package frc.robot.subsystems.arm;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.signals.*;
import edu.wpi.first.wpilibj.DigitalInput;

public class ArmIOReal implements ArmIO {

    private final TalonFX armMotor;
    private final DigitalInput stowedLimit;

    private final MotionMagicVoltage mmRequest = new MotionMagicVoltage(0);
    private final DutyCycleOut dutyRequest = new DutyCycleOut(0);

    public ArmIOReal() {
        armMotor = new TalonFX(frc.robot.Constants.ArmConstants.armMotorPort);
        stowedLimit = new DigitalInput(frc.robot.Constants.ArmConstants.stowedLimitSwitch);

        var config = new TalonFXConfiguration();

        // MotionMagic
        config.MotionMagic.MotionMagicCruiseVelocity = 30000;
        config.MotionMagic.MotionMagicAcceleration = 24000;

        // PID (slot 0)
        config.Slot0.kP = 0.275;
        config.Slot0.kI = 0.0;
        config.Slot0.kD = 0.0;
        config.Slot0.kV = 0.0;

        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 12.156;
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 1.204;


        armMotor.getConfigurator().apply(config);
    }

    @Override
    public void updateInputs(ArmIOInputs inputs) {
        var s = armMotor.getPosition();
        inputs.positionRotations = s.getValueAsDouble();

        var v = armMotor.getVelocity();
        inputs.velocityRps = v.getValueAsDouble();

        inputs.statorCurrent = armMotor.getStatorCurrent().getValueAsDouble();
        inputs.stowedLimitPressed = !stowedLimit.get();
        inputs.isMotionDone = armMotor.getClosedLoopError().getValueAsDouble() < 200;
    }

    @Override
    public void setPercent(double percent) {
        armMotor.setControl(dutyRequest.withOutput(percent));
    }

    @Override
    public void setMotionMagic(double positionRotations) {
        armMotor.setControl(mmRequest.withPosition(positionRotations));
    }

    @Override
    public void resetEncoder() {
        armMotor.setPosition(0);
    }

    public void stop(){
        armMotor.setControl(new DutyCycleOut(0));
    }
}
