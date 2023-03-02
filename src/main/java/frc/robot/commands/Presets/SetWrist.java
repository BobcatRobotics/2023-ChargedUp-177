package frc.robot.commands.Presets;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Wrist;

public class SetWrist extends CommandBase {
    Wrist wrist;
    boolean direction;
    public SetWrist(Wrist w, boolean d ) {
        wrist = w;
        direction = d;
    }
  
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

    }
  
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute(){
        if (direction){
            wrist.wristSolenoidON();
        }
        else{
            wrist.wristSolenoidOFF();
        }
    }
  
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }
  
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        if(wrist.getWristSolenoid() == direction){
            return true;
        }
        return false;
    } 

}
