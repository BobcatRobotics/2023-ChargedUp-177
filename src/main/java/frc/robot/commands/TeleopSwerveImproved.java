package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;


public class TeleopSwerveImproved extends CommandBase {  
    

    private Swerve s_Swerve;    
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup; //rotation joystick rotational axis
    private DoubleSupplier angleXSup; //rotation joystick x
    private DoubleSupplier angleYSup; //rotation joystick y
    private BooleanSupplier robotCentricSup;
    PIDController pid = new PIDController(0.6, 0, 0); //TODO: Tune


    public TeleopSwerveImproved(Swerve s_Swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup, DoubleSupplier angleXSup, DoubleSupplier angleYSup, BooleanSupplier robotCentricSup) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.angleXSup = angleXSup;
        this.angleYSup = angleYSup;
        this.robotCentricSup = robotCentricSup;
    }


    @Override
    public void execute() {
        /* Get Values, Deadband*/
        double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband);
        double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband);
        double rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), 0.0); //TODO: add support for rotational axis
        SmartDashboard.putNumber("rotationVal", rotationVal);
        double angleXVal = MathUtil.applyDeadband(angleXSup.getAsDouble(), 0.3);
        double angleYVal = MathUtil.applyDeadband(angleYSup.getAsDouble(), 0.3);
        double currentHeading = s_Swerve.getYaw().getDegrees()%360;
        
        double desiredHeading = ((Math.atan2(angleYVal, angleXVal) * 180 / Math.PI)+360)%360;
        
        double distance = (Math.max(currentHeading, desiredHeading)-Math.min(currentHeading, desiredHeading));
        
        if (Math.abs(desiredHeading - currentHeading) < 0.25) {
            desiredHeading = currentHeading;
            SmartDashboard.putBoolean("is in range", true);
        }
        if (Math.abs(angleXVal) < 0.3 && Math.abs(angleYVal) < 0.3) {
            desiredHeading = currentHeading;
        }
        
        
        double rot = pid.calculate(currentHeading, distance)/75;


        /* Drive */
        s_Swerve.drive(
            new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed), 
            rot * Constants.Swerve.maxAngularVelocity, 
            !robotCentricSup.getAsBoolean(), 
            true
        );
        SmartDashboard.putNumber("error", pid.getPositionError());
        SmartDashboard.putNumber("desiredHeading", desiredHeading);
        SmartDashboard.putNumber("currentHeading", currentHeading);
        SmartDashboard.putBoolean("is at setpoint", pid.atSetpoint());
        SmartDashboard.putNumber("Rot", rot);
        
    }
}