package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {
    private WPI_TalonFX armMotor;
    private PIDController armController;
    
    public Arm() {
        armMotor = new WPI_TalonFX(Constants.ArmConstants.armMotorPort);
        armController = new PIDController(0.3, 0, 0); // TODO: Tune!

        armMotor.configFactoryDefault();
        armMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 20);
        armMotor.setSensorPhase(true);
        armMotor.configNominalOutputForward(0, 20);
        armMotor.configNominalOutputReverse(0, 20);
        armMotor.configPeakOutputForward(1, 20);
        armMotor.configPeakOutputReverse(-1, 20);
        armMotor.configAllowableClosedloopError(0, 0, 20);
        armMotor.config_kF(0, 0, 20);
        armMotor.config_kP(0, 0.1, 20);
        armMotor.config_kI(0, 0, 20);
        armMotor.config_kD(0, 0.5, 20);
    
    }

    public void setSpeed(double speed) {
        armMotor.set(ControlMode.PercentOutput, speed);
    }

    public void setState(int state) {
        double output;
        if (state == 0) {
            armMotor.set(TalonFXControlMode.Position, Constants.ArmConstants.pos1);
        } else if (state == 1) {
            armMotor.set(TalonFXControlMode.Position, Constants.ArmConstants.pos2);
        } else {
            armMotor.set(TalonFXControlMode.Position, Constants.ArmConstants.pos3);
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
