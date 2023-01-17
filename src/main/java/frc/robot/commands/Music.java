package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.music.Orchestra;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;;
public class Music extends CommandBase {
  Swerve driveTrain;
  //create our orchestra
  Orchestra orchestra = new Orchestra();
  //create an array of all our motors
  TalonFX[] motors;
  
  private boolean isPlaying = false;
  private boolean finished = false;
  

  BooleanSupplier startPlaying;

    /**
     * src/main/java/frc/robot/Music_Files/SELECTED_MUSIC_FILE.chrp
     */
    public Music(Swerve driveTrain, String path, BooleanSupplier startButton) {
     this.driveTrain = driveTrain;
     this.startPlaying = startButton;

     //create an array of all our motors, not the best way to do this but idk how to do it better
     //arrays are weird
      motors = new TalonFX[] {
        driveTrain.getMod0Motors()[0],
        driveTrain.getMod0Motors()[1],
        driveTrain.getMod1Motors()[0],
        driveTrain.getMod1Motors()[1],
        driveTrain.getMod2Motors()[0],
        driveTrain.getMod2Motors()[1],
        driveTrain.getMod3Motors()[0],
        driveTrain.getMod3Motors()[1]
      };

      //add each motor to our orchestra
      for (TalonFX motor : motors) {
        orchestra.addInstrument(motor);
      }
     
     //load our music file
    orchestra.loadMusic(path);
      
    }


    
  
  

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //play our music
  if(startPlaying.getAsBoolean()){
    if (!isPlaying) {
      orchestra.play();
      isPlaying = true;
    }
  }
    if (orchestra.isPlaying() == false) {
      finished = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //stop our music
    orchestra.stop();
    isPlaying = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}