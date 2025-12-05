package frc.robot.subsystems.wrist;

public class WristIOSim implements WristIO {

    private boolean solenoid = false;
    private double pressure = 100.0;

    @Override
    public void updateInputs(WristIOInputs inputs) {
        inputs.solenoidExtended = solenoid;
        inputs.pressurePsi = pressure;
        inputs.compressorEnabled = true;
    }

    @Override
    public void setSolenoid(boolean extended) {
        solenoid = extended;
    }

    @Override
    public void enableCompressorAnalog(double minPsi, double maxPsi) {
        // simulate pressure rising slowly
        if (pressure < maxPsi) {
            pressure += 0.5;
        }
        if (pressure > maxPsi) {
            pressure = maxPsi;
        }
    }
}
