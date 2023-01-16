package frc.robot.commands;

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
  private boolean isPlaying;

    /**
     * src/main/java/frc/robot/Music_Files/SELECTED_MUSIC_FILE.chrp
     */
    public Music(Swerve driveTrain, String path) {
     this.driveTrain = driveTrain;
     motors = driveTrain.getMotors();

     //for each motor in our array
     for(TalonFX motor : motors){
      //add it motor to our orchestra
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
    if (!isPlaying) {
      orchestra.play();
      isPlaying = true;
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
    return false;
  }
}