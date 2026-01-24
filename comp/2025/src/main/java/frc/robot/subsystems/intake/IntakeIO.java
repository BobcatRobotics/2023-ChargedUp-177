package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    @AutoLog
    public static class IntakeIOInputs {
        public double statorCurrent = 0.0;
        public double velocityRps = 0.0;
        public double percentOutput = 0.0;
    }

    public default void updateInputs(IntakeIOInputs inputs) {}

    public default void setPercent(double percent) {}

    public default void stop() {
        setPercent(0);
    }
}

