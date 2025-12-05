package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class Arm extends SubsystemBase {

    private final ArmIO io;
    private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();

    public Arm(ArmIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);

        // Log all inputs
        Logger.processInputs("Arm", inputs);

        // Auto-reset when stowed limit hit
        if (inputs.stowedLimitPressed) {
            io.resetEncoder();
        }
    }

    public void setPercent(double speed) {
        io.setPercent(speed);
    }

    public void goToState(int state) {
        switch (state) {
            case 0 -> io.setMotionMagic(ArmConstants.pos0);
            case 1 -> io.setMotionMagic(ArmConstants.pos1);
            case 2 -> io.setMotionMagic(ArmConstants.pos2);
            case 3 -> io.setMotionMagic(ArmConstants.bottomPickup);
        }
    }

    public void setPositionRotations(double pos) {
        io.setMotionMagic(pos);
    }

    public boolean isAtSetpoint() {
        return inputs.isMotionDone;
    }

    public boolean isAtTopLimit() {
        return inputs.positionRotations >= ArmConstants.topLimit;
    }

    public boolean isAtBottomLimit() {
        return inputs.positionRotations <= ArmConstants.bottomLimit;
    }

    public double getPosition() {
        return inputs.positionRotations;
    }
}
