package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {
    private TalonFX armMotor;
    private PIDController armController;
    
    public Arm() {
        armMotor = new TalonFX(Constants.ArmConstants.armMotorPort);
        armController = new PIDController(0.3, 0, 0); // TODO: Tune!
    }

    public void setSpeed(double percent) {
        armMotor.set(TalonFXControlMode.PercentOutput, percent);
    }

    public void setState(int state) {
        double output;
        if (state == 0) {
            output = armController.calculate(armMotor.getSelectedSensorPosition(), Constants.ArmConstants.pos1);
            armMotor.set(TalonFXControlMode.Position, output);
        } else if (state == 1) {
            output = armController.calculate(armMotor.getSelectedSensorPosition(), Constants.ArmConstants.pos2);
            armMotor.set(TalonFXControlMode.Position, output);
        } else {
            output = armController.calculate(armMotor.getSelectedSensorPosition(), Constants.ArmConstants.pos3);
            armMotor.set(TalonFXControlMode.Position, output);
        }
    }

    public boolean isAtLimits() {
        return (armMotor.getSelectedSensorPosition() <= Constants.ArmConstants.bottomLimit && armMotor.getSelectedSensorPosition() >= Constants.ArmConstants.topLimit);
    }

    public boolean isAtTopLimit() {
        return armMotor.getSelectedSensorPosition() >= Constants.ArmConstants.topLimit;
    }

    public boolean isAtBottomLimit() {
        return armMotor.getSelectedSensorPosition() <= Constants.ArmConstants.bottomLimit;
    }
    
}
