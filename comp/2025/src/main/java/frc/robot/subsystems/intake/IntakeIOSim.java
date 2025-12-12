package frc.robot.subsystems.intake;

public class IntakeIOSim implements IntakeIO {
    private double percent = 0;

    public IntakeIOSim(){        
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.percentOutput = percent;
        inputs.velocityRps = percent * 20; // fake sim speed
        inputs.statorCurrent = Math.abs(percent) * 10;
    }

    @Override
    public void setPercent(double percent) {
        this.percent = percent;
    }
}
