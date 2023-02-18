// Intake subsystem code for an FRC robot
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of the WPILib BSD license file

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Intake subsystem for an FRC robot. 
 * The class contains methods for controlling the intake mechanism's motors.
 */
public class Intake extends SubsystemBase {

    // Speed at which the intake motor should run when moving forward
    private final double inSpeed;
    // Speed at which the intake motorsshould run when moving in reverse
    private final double outSpeed;
    
    // CAN IDs for the motor
    public static final int topMotorPort = 11;
    
    
    //intake motor objects
    private WPI_TalonFX topMotor;
   
    
    /**
     * Constructor for the Intake class.
     * Initializes the top and bottom motor objects and sets the speeds for the intake motors.
     */
    public Intake(){
      topMotor =  new WPI_TalonFX(topMotorPort);
      
      inSpeed = 1.0;
      outSpeed = 0.50;
    }
    
    /**
     * Method to run the intake motors forward.
     */
    public void intakeForward() {
        topMotor.set(inSpeed);
        
    }
    
    /**
     * Method to run the intake motors in reverse.
     */
    public void reverseIntake() {
        topMotor.set(outSpeed * -1);
       
    }
    
    /**
     * Method to stop the intake motors.
     */
    public void intakeStop() {
        topMotor.set(0);
        
    }
}
