package frc.robot.subsystems.wrist;

import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.units.measure.Angle;
import frc.robot.Constants;

public enum WristState {
  UNKNOWN(-1),
  BOTTOM(Constants.WristConstants.bottomLimit),
  TOP(Constants.WristConstants.topLimit);

  WristState(double rotations) {
    this.position = Rotations.of(rotations);
  }

  public Angle position;
}
