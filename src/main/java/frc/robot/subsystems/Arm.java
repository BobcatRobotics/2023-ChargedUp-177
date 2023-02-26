package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXSensorCollection;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;

public class Arm extends SubsystemBase {
    private WPI_TalonFX armMotor;
    //private TalonFXSensorCollection absoluteEncoder;
    private DigitalInput armLimit;

    // private double holdPosValue;
    
    public Arm() {
        armMotor = new WPI_TalonFX(Constants.ArmConstants.armMotorPort);
        armLimit = new DigitalInput(Constants.ArmConstants.stowedLimitSwitch);
        //absoluteEncoder = new TalonFXSensorCollection(armMotor);

        armMotor.configFactoryDefault();
        armMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 20);
        armMotor.setSensorPhase(true);
        armMotor.configNominalOutputForward(0, 20);
        armMotor.configNominalOutputReverse(0, 20);
        armMotor.configPeakOutputForward(0.5, 20);
        armMotor.configPeakOutputReverse(-0.5, 20);
        armMotor.configAllowableClosedloopError(0, 0, 20);
        armMotor.config_kF(0, 0, 20);
        armMotor.config_kP(0, 0.275, 20);
        armMotor.config_kI(0, 0, 20);
        armMotor.config_kD(0, 0, 20);
        armMotor.setInverted(true);
        armMotor.setNeutralMode(NeutralMode.Brake);

        // holdPosValue = armMotor.getSelectedSensorPosition();
    }

    public void setSpeed(double speed) {
        armMotor.set(ControlMode.PercentOutput, speed);
    }

    // public void holdPosition() {
    //     armMotor.set(ControlMode.Position, holdPosValue);
    // }

    // public void setHoldPos() {
    //     holdPosValue = armMotor.getSelectedSensorPosition();
    // }

    public void setState(int state) {
        if (state == 0) {
            armMotor.set(TalonFXControlMode.Position, ArmConstants.pos0);
        } else if (state == 1) {
            armMotor.set(TalonFXControlMode.Position, ArmConstants.pos1);
        } else  if (state == 2) {
            armMotor.set(TalonFXControlMode.Position, ArmConstants.pos2);
        } else if (state == 3) {
            armMotor.set(TalonFXControlMode.Position, ArmConstants.bottomPickup);
        }
    }

    public boolean isAtStowedLimit() {
        return !armLimit.get();
    }

    public int getState() {
        double pos = armMotor.getSelectedSensorPosition();
        if (pos <= 256) pos = 0;
        return (int) Math.ceil(pos/4096);
    }

    public double getPos() {
        return armMotor.getSelectedSensorPosition();
    }

    public boolean isAtTopLimit() {
        return armMotor.getSelectedSensorPosition() >= Constants.ArmConstants.topLimit;
    }

    public boolean isAtBottomLimit() {
        return armMotor.getSelectedSensorPosition() <= Constants.ArmConstants.bottomLimit;
    }

    public boolean isAtConstrictedBottomLimit() {
        return armMotor.getSelectedSensorPosition() <= Constants.ArmConstants.constrictedBottomLimit;
    }

    // public double absoluteEncoderVal() {
    //     return absoluteEncoder.getIntegratedSensorAbsolutePosition();
    // }
    
    public boolean isAtHardStop() {
        return armMotor.getStatorCurrent() >= 35.0;
    }

    public void resetEncoder() {
        armMotor.setSelectedSensorPosition(0);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        if (isAtStowedLimit()) {
            resetEncoder();
        }
    }
}
