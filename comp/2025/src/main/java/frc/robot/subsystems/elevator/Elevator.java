package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class Elevator extends SubsystemBase {

    private final ElevatorIO io;
    private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

    private double holdPosValue = 0;

    public Elevator(ElevatorIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Elevator", inputs);

        // Auto-reset bottom limit
        if (inputs.bottomLimitPressed) {
            io.resetEncoder();
            holdPosValue = 0;
            io.setPercent(0);
        }
    }

    public void elevate(double speed) {
        io.setPercent(speed);
    }

    public double getSpeed() {
        return inputs.velocityRps;
    }

    public boolean getStopped() {
        return getSpeed() == 0;
    }

    public void holdPosition() {
        io.holdPosition(holdPosValue);
    }

    public void holdPosition(double pos) {
        io.holdPosition(pos);
    }

    public double getPIDError() {
        return inputs.positionRotations - holdPosValue;
    }

    public void setHoldPos() {
        holdPosValue = inputs.positionRotations;
    }

    public boolean isAtSetpoint() {
        return inputs.isMotionDone;
    }

    public boolean getBottomLimits() {
        return inputs.bottomLimitPressed;
    }

    public boolean topLimitSwitch() {
        return inputs.topLimitPressed;
    }

    public double getEncoder() {
        return inputs.positionRotations;
    }

    public boolean isAtCurrentLimit() {
        return inputs.statorCurrent >= 50.0;
    }

    public void resetEncoderTop() {
        io.resetEncoderTop();
    }

    public void setState(double pos) {
        holdPosValue = pos;
        io.setMotionMagic(pos);
        Logger.recordOutput("Elevator/Setpoint", pos);
    }
}
