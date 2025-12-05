package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
    @AutoLog
    public static class ArmIOInputs {
        public double positionRotations = 0.0;
        public double velocityRps = 0.0;
        public double statorCurrent = 0.0;
        public boolean stowedLimitPressed = false;
        public boolean isMotionDone = false;
    }

    public default void updateInputs(ArmIOInputs inputs) {}

    public default void setPercent(double percent) {}
    public default void setMotionMagic(double position) {}

    public default void resetEncoder() {}
}
