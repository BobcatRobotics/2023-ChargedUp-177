package frc.robot.subsystems.wrist;

import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.units.measure.Angle;

public enum WristState {
  UNKNOWN(-1);

  WristState(double rotations) {
    this.position = Rotations.of(rotations);
  }

  public Angle position;
}
