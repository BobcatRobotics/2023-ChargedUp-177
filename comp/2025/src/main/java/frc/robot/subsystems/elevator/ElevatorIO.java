package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
    @AutoLog
    public static class ElevatorIOInputs {
        public double positionRotations = 0.0;
        public double velocityRps = 0.0;
        public double statorCurrent = 0.0;
        public boolean bottomLimitPressed = false;
        public boolean topLimitPressed = false;
        public boolean isMotionDone = false;
    }

    public default void updateInputs(ElevatorIOInputs inputs) {
    }

    public default void setPercent(double percent) {
    }

    public default void setMotionMagic(double position) {
    }

    public default void holdPosition(double position) {
    }

    public default void resetEncoder() {
    }

    public default void resetEncoderTop() {
    }
}
