package frc.robot.subsystems.wrist;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Wrist extends SubsystemBase {

    private final WristIO io;
    private final WristIOInputsAutoLogged inputs = new WristIOInputsAutoLogged();

    public Wrist(WristIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);

        // Log with AdvantageKit
        Logger.processInputs("Wrist", inputs);

        // Keep compressor in analog mode (same as your code)
        io.enableCompressorAnalog(80, 115);
    }

    public void wristSolenoidON() {
        io.setSolenoid(true);
    }

    public void wristSolenoidOFF() {
        io.setSolenoid(false);
    }

    public boolean getWristSolenoid() {
        return inputs.solenoidExtended;
    }

    public double getPressure() {
        return inputs.pressurePsi;
    }
}
