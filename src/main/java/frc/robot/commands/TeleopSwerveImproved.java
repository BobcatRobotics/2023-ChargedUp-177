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
    private double angleXVal;
    private double angleYVal;
    private double currentHeading;
    private double translationVal;
    private double strafeVal;
    private double rotationVal;
    private double desiredHeading;
    private double distance;
    double rot;

    private double deadBand = 0.3;    

    private double kP = 0.4;
    private double kI = 0.1;
    private double kD = 0;
    PIDController pid = new PIDController(kP, kI, kD); //TODO: Tune

    /**
     * this will turn to the direction the joystick is pointing,
     * instead of rotating at a constant rate
     * 
     * @param s_Swerve The swerve subsystem
     * @param translationSup The translation joystick axis (forward/backward)
     * @param strafeSup The strafe joystick axis (left/right)
     * @param rotationSup The rotation joystick axis (rotate at a constant rate supplied by the joystick)
     * @param angleXSup The rotation joystick x axis (rotate to a specific angle)
     * @param angleYSup The rotation joystick y axis (rotate to a specific angle)
     * @param robotCentricSup Whether or not the robot is in robot centric mode
     */
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
    public void updateShuffleBoardDebugging(){
        SmartDashboard.putNumber("error", pid.getPositionError());
        SmartDashboard.putNumber("desiredHeading", desiredHeading);
        SmartDashboard.putNumber("currentHeading", currentHeading);
        SmartDashboard.putBoolean("is at setpoint", pid.atSetpoint());
        SmartDashboard.putNumber("Rot", rot);
        SmartDashboard.putNumber("x", angleXVal);
        SmartDashboard.putNumber("y", angleYVal);
    }

    @Override
    public void execute() {
        /* Get Values, Deadband*/
         translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband);
         strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband);
         rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), 0.0); //TODO: add support for rotational axis
        SmartDashboard.putNumber("rotationVal", rotationVal);
         angleXVal = MathUtil.applyDeadband(angleXSup.getAsDouble(), deadBand);
         angleYVal = MathUtil.applyDeadband(angleYSup.getAsDouble(), 0.3);
         currentHeading = s_Swerve.getYaw().getDegrees()%360;
        

        /* Calculate desired heading based on joystick input,
         * for example, if the joystick is pointing straight up,
         * the desired heading is 0 degrees, if it is pointing
         * to the top right, the desired heading is 45 degrees, etc
         */
        //TODO: fix angle offset
        desiredHeading = ((Math.atan2(angleYVal, angleXVal) * 180 / Math.PI)+270)%360;
        
        updateShuffleBoardDebugging();

        //find the shortest distance between the current heading and the desired heading
        distance = (Math.max(currentHeading, desiredHeading)-Math.min(currentHeading, desiredHeading));
        //if the current heading is greater than the desired heading, turn counterclockwise
        distance = (currentHeading > desiredHeading) ? -distance : distance;

        //if the desired heading is within 0.25 degrees of the current heading,
        //stay where we are
        if (Math.abs(desiredHeading - currentHeading) < 0.25) {
            desiredHeading = currentHeading;
            SmartDashboard.putBoolean("is in range", true);
        }
        else {
            SmartDashboard.putBoolean("is in range", false);
        }
        //if the joystick is within the deadband, stay where we are
        if (Math.abs(angleXVal) < deadBand && Math.abs(angleYVal) < deadBand) {
            desiredHeading = currentHeading;
        }
        
        
        rot = rotationVal == 0 ? (pid.calculate(currentHeading, distance)/75) : ((pid.calculate(currentHeading, distance)/75)+rotationVal)/2 ;


        /* Drive */
        s_Swerve.drive(
            new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed), 
            rot * Constants.Swerve.maxAngularVelocity, 
            !robotCentricSup.getAsBoolean(), 
            true
        );
    }
}