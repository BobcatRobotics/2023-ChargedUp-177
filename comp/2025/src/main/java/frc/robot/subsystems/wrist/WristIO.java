package frc.robot.subsystems.wrist;

import org.littletonrobotics.junction.AutoLog;

public interface WristIO {
  @AutoLog
  public static class WristIOInputs {
    public double absolutePositionDeg = 0.0;
    public double motorVelocity = 0.0;
    public double motorOutput = 0.0;
  }

  public default void updateInputs(WristIOInputs inputs) {
  }

  public default void setPercent(double percent) {
  }

  public default void setMotionMagic(WristState state) {
  }

  public default void stop() {
  }
}
