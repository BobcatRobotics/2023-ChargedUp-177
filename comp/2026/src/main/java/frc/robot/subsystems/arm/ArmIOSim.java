package frc.robot.subsystems.arm;

public class ArmIOSim implements ArmIO {

    private double percent = 0;

    @Override
    public void updateInputs(ArmIOInputs inputs) {
        inputs.velocityRps = percent * 20; // fake sim speed
        inputs.statorCurrent = Math.abs(percent) * 10;
    }

    @Override
    public void setPercent(double percent) {
        this.percent = percent;
    }
}
