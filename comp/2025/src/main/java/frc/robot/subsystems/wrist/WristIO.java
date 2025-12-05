package frc.robot.subsystems.wrist;

import org.littletonrobotics.junction.AutoLog;

public interface WristIO {
    @AutoLog
    public static class WristIOInputs {
        public boolean solenoidExtended = false;
        public double pressurePsi = 0.0;
        public boolean compressorEnabled = false;
    }

    public default void updateInputs(WristIOInputs inputs) {}

    public default void setSolenoid(boolean extended) {}

    public default void enableCompressorAnalog(double minPsi, double maxPsi) {}
}
