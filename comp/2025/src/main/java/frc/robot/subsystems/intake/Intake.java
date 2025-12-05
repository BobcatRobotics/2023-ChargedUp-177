package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.MathUtils;

public class Intake extends SubsystemBase {

    private final IntakeIO io;
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

    public Intake(IntakeIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);
    }

    public void runIntakeIn() {
        io.setPercent(-0.9);
    }

    public void runIntakeOut() {
        io.setPercent(0.4);
    }

    public void runIntakeOutFull() {
        io.setPercent(1.0);
    }

    public void runIntakePercent(double speed) {
        io.setPercent(MathUtils.throttlePercent(speed));
    }

    public void stop() {
        io.setPercent(0.0);
    }

    public boolean isAtHardStop() {
        return inputs.statorCurrent >= 20.0;
    }
}
