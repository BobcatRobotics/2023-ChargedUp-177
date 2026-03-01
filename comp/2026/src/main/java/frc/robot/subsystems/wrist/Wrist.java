package frc.robot.subsystems.wrist;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.AutoLog;

public class Wrist extends SubsystemBase {
  private final WristIO io;
  private final WristIOInputsAutoLogged inputs = new WristIOInputsAutoLogged();

  public Wrist(WristIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);

    Logger.processInputs("Wrist", inputs);
  }

  /** Get absolute wrist position (°) */
  public double getWristPositionDeg() {
    return inputs.absolutePositionDeg;
  }

  /** Manual control */
  public void setSpeed(double speed) {
    io.setPercent(speed);
  }

  public void getOutofTheWay() {
    io.getOutofTheWay();
  }

  /** Stop */
  public void stop() {
    io.stop();
  }

  /** Set wrist to preset position */
  public void setState(WristState state) {
    io.setMotionMagic(state);
  }


  /** Soft limit check using absolute encoder */
  public boolean topLimit() {
    return getWristPositionDeg() > Constants.WristConstants.topLimit;
  }
}