package frc.robot.subsystems.wrist;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.units.Units;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;

public class WristIOSim implements WristIO {

  private final TalonFX motor = new TalonFX(0); // device ID doesn't matter in sim
  private final CANcoder encoder = new CANcoder(0);

  private final TalonFXSimState motorSim = motor.getSimState();
  private final CANcoderSimState encoderSim = encoder.getSimState();

  // reused control objects
  private final DutyCycleOut percentReq = new DutyCycleOut(0);
  private final MotionMagicVoltage mmReq = new MotionMagicVoltage(0);

  // internal sim state
  private double simulatedPositionRot = 0;   // motor rotations
  private double simulatedVelocityRotPerSec = 0;

  public WristIOSim() {
    motorSim.setSupplyVoltage(12.0);
    encoderSim.setSupplyVoltage(12.0);
  }

  @Override
  public void updateInputs(WristIOInputs inputs) {

    // Integrate velocity into position (simple physics)
    simulatedPositionRot += simulatedVelocityRotPerSec * 0.02;

    // update encoder to match simulated position
    encoderSim.setRawPosition(simulatedPositionRot);
    encoderSim.setVelocity(simulatedVelocityRotPerSec);

    inputs.absolutePositionDeg = encoder.getAbsolutePosition().getValueAsDouble();
    inputs.motorVelocity = motor.getVelocity().getValueAsDouble();
    inputs.motorOutput = motor.getDutyCycle().getValueAsDouble();

    // Feed Phoenix 6 simulation state machine
    motorSim.setRotorVelocity(simulatedVelocityRotPerSec);
    motorSim.setRawRotorPosition(simulatedPositionRot);
  }

  @Override
  public void setPercent(double percent) {
    percentReq.Output = percent;
    motor.setControl(percentReq);

    simulatedVelocityRotPerSec = percent * 50;  // approx free speed
  }

  @Override
  public void setMotionMagic(WristState state) {
    motor.setControl(mmReq.withPosition(state.position));

    // crude Motion Magic simulation:
    double error = (state.position.in(Units.Rotations) - simulatedPositionRot);
    simulatedVelocityRotPerSec = error * 5; // proportional approximation
  }

  @Override
  public void stop() {
    motor.setControl(new DutyCycleOut(0));
    simulatedVelocityRotPerSec = 0;
  }
}