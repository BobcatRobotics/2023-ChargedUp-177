package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.units.measure.Angle;

public enum ArmState {
    ArmState(-1);

    ArmState(double rotations) {
      this.position = Rotations.of(rotations);
    }
  
    public Angle position;
  }