package frc.robot.subsystems.wrist;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Solenoid;
import frc.robot.Constants;

public class WristIOReal implements WristIO {

    private final Solenoid solenoid;
    private final PneumaticHub hub;
    private final Compressor compressor;

    public WristIOReal() {
        hub = new PneumaticHub(Constants.pHubID);
        solenoid = new Solenoid(PneumaticsModuleType.REVPH, Constants.wristSolenoidID);
        compressor = new Compressor(Constants.compressorID, PneumaticsModuleType.REVPH);
    }

    @Override
    public void updateInputs(WristIOInputs inputs) {
        inputs.solenoidExtended = solenoid.get();
        inputs.pressurePsi = compressor.getPressure();
        inputs.compressorEnabled = compressor.isEnabled();
    }

    @Override
    public void setSolenoid(boolean extended) {
        solenoid.set(extended);
    }

    @Override
    public void enableCompressorAnalog(double minPsi, double maxPsi) {
        compressor.enableAnalog(minPsi, maxPsi);
    }
}
