package frc.robot.subsystems.elevator;

public class ElevatorIOSim implements ElevatorIO {

    private double position = 0;
    private double velocity = 0;

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        inputs.positionRotations = position;
        inputs.velocityRps = velocity;
        inputs.statorCurrent = Math.abs(velocity) * 0.1;
        inputs.bottomLimitPressed = position <= 0;
        inputs.topLimitPressed = position >= 100000; // fake top
        inputs.isMotionDone = Math.abs(velocity) < 1;
    }

    @Override
    public void setPercent(double percent) {
        velocity = percent * 5000;
        position += velocity * 0.02;
    }

    @Override
    public void setMotionMagic(double pos) {
        velocity = (pos - position) * 0.1;
        position += velocity * 0.02;
    }

    @Override
    public void holdPosition(double pos) {
        setMotionMagic(pos);
    }

    @Override
    public void resetEncoder() {
        position = 0;
    }

    @Override
    public void resetEncoderTop() {
        position = -236710;
    }
}
