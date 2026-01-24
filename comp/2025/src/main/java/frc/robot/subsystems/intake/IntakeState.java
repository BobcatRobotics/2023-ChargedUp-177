package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.units.measure.Angle;

public enum IntakeState {
    UNKNOWN(-1);

    IntakeState(double rotations) {
      this.position = Rotations.of(rotations);
    }
  
    public Angle position;
  }