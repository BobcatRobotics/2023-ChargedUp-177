// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import frc.robot.Constants;
import edu.wpi.first.math.trajectory.TrapezoidProfile;




public class Elevator extends SubsystemBase {

  private DigitalInput topLimit;
  private DigitalInput bottomLimit;
  //private DigitalInput middleLimit;
  private WPI_TalonFX elevator_motor;
  private Encoder encoder;
  private int state;
  private PIDController elevatorCONTROLLER;
  private double tolerance = 0.2;



  public Elevator() { //gear reduction is 15 to 1

    /* Inits the limits (poet frfr) + Also initializes the elevator motor*/
    topLimit = new DigitalInput(Constants.elevator.toplimitSwitchPWMport);
    bottomLimit = new DigitalInput(Constants.elevator.bottomlimitSwitchPWMport);
    elevator_motor = new WPI_TalonFX(Constants.elevator.Elevator_motorCanID);

    /*
    (!!)All encoder stuff is commented out for now(!!)

    encoder = new Encoder(topLimit, bottomLimit);
    */

    //Creates PID for elevator
    elevatorCONTROLLER = new PIDController(0, 0, 0);
    //encoder.reset();
    elevatorCONTROLLER.reset();
    elevatorCONTROLLER.setTolerance(tolerance);
  }

  /* Returns the value of limits */
  public boolean getTopLimit() {
    return !topLimit.get();
  }
  public boolean getBottomLimit() {
    return !bottomLimit.get();
  }

  
  public void stop() {
    elevator_motor.stopMotor();
  }
  public void elevatorMotorset(double speed_amount){
    elevator_motor.set(speed_amount);
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public void setInvertedtrue() {
    elevator_motor.setInverted(true);
  }
  public void setInvertedfalse(){
    elevator_motor.setInverted(false);
  }

  /* Returns the position of the elevator (whether it's at the top limit (1), bottom limit (0), or in the middle (-1)) */
  public int get_state(){
    return(state);
  }


  /* Encoder and PID calculations */
  public double get_encoder_distance(){
    return(elevator_motor.getSelectedSensorPosition());
  }
  public void reset_encoder(){
    encoder.reset();
  }
  
  public void setpidpoint(double Setpoint){
    elevatorCONTROLLER.setSetpoint(Setpoint);
  }
  public double calculate(double current_position, double wanted_position){
    return(elevatorCONTROLLER.calculate(current_position, wanted_position));
  }
}
