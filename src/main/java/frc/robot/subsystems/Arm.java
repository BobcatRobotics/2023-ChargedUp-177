package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ElevatorConstants;

public class Arm extends SubsystemBase {
    private TalonFX armMotor;
    //private TalonFXSensorCollection absoluteEncoder;
    private DigitalInput armLimit;

final MotionMagicVoltage m_motmag = new MotionMagicVoltage(0);
    // private double holdPosValue;
    
    public Arm() {
        armMotor = new TalonFX(Constants.ArmConstants.armMotorPort);
        armLimit = new DigitalInput(Constants.ArmConstants.stowedLimitSwitch);
        //absoluteEncoder = new TalonFXSensorCollection(armMotor);

        TalonFXConfiguration internalConfig =  new TalonFXConfiguration();
        internalConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        Slot0Configs slot0Configs = new Slot0Configs();
        slot0Configs.kP =  0.275;
        slot0Configs.kI =  0;
        slot0Configs.kD =  0;
        internalConfig.withSlot0(slot0Configs);
        internalConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        internalConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        internalConfig.MotionMagic.MotionMagicCruiseVelocity = 30000;
        internalConfig.MotionMagic.MotionMagicAcceleration = 24000;
        armMotor.getConfigurator().apply(internalConfig);
    }

    public void setSpeed(double speed) {
        armMotor.set(speed);
    }

    // public void holdPosition() {
    //     armMotor.set(ControlMode.Position, holdPosValue);
    // }

    // public void setHoldPos() {
    //     holdPosValue = armMotor.getPosition().getValueAsDouble();
    // }

    public void setState(int state) {
        if (state == 0) {

            armMotor.setControl(m_motmag.withPosition(ArmConstants.pos0));
            SmartDashboard.putString("arm error", "State: " + state + ", Error: " + getArmPIDError());
        } else if (state == 1) {

            armMotor.setControl(m_motmag.withPosition(ArmConstants.pos1));
            SmartDashboard.putString("arm error", "State: " + state + ", Error: " + getArmPIDError());
        } else  if (state == 2) {

            armMotor.setControl(m_motmag.withPosition(ArmConstants.pos2));
            SmartDashboard.putString("arm error", "State: " + state + ", Error: " + getArmPIDError());
        } else if (state == 3) {
            armMotor.setControl(m_motmag.withPosition(ArmConstants.bottomPickup));
            SmartDashboard.putString("arm error", "State: " + state + ", Error: " + getArmPIDError());
        }
    }
    public void setPos(int pos) {
        armMotor.setControl(m_motmag.withPosition(pos));
        SmartDashboard.putString("arm error", "State: " + pos + ", Error: " + getArmPIDError());
    }
    public double getArmPIDError(){
        return armMotor.getClosedLoopError().getValueAsDouble();
    }
    public boolean isAtStowedLimit() {
        return !armLimit.get();
    }

    public int getState() {
        double pos = armMotor.getPosition().getValueAsDouble();
        if (pos <= 256) pos = 0;
        return (int) Math.ceil(pos/4096);
    }

    public double getPos() {
        return armMotor.getPosition().getValueAsDouble();
    }

    public boolean isAtTopLimit() {
        return armMotor.getPosition().getValueAsDouble() >= Constants.ArmConstants.topLimit;
    }

    public boolean isAtBottomLimit() {
        return armMotor.getPosition().getValueAsDouble() <= Constants.ArmConstants.bottomLimit;
    }

    public boolean isAtConstrictedBottomLimit() {
        return armMotor.getPosition().getValueAsDouble() <= Constants.ArmConstants.constrictedBottomLimit;
    }

    public boolean isAtSetpoint() {
        double error = armMotor.getClosedLoopError().getValueAsDouble();
        double threshold = 0.5;
        if( error <= threshold){
          return true;
        }
        return false;
      }
    
    public boolean isAtCurrentLimit() {
        double statorCurrent = armMotor.getStatorCurrent().getValueAsDouble();
        return statorCurrent >= 50.0;
      }

    public void resetEncoder() {
        armMotor.setPosition(0);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        if (isAtStowedLimit()) {
            resetEncoder();
        }
    }
}
