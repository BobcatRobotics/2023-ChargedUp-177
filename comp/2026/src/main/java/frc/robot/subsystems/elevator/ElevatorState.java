package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.units.measure.Angle;

public enum ElevatorState {
    UNKNOWN(-1);

    ElevatorState(double rotations) {
      this.position = Rotations.of(rotations);
    }
  
    public Angle position;
  }