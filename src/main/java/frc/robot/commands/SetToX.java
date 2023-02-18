package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;

public class SetToX extends CommandBase {
    private Swerve drivetrain;
    
    public SetToX(Swerve dt) {
        drivetrain = dt;
        addRequirements(dt);
    }

    @Override
    public void initialize() {
        drivetrain.configToX();
        drivetrain.drive(new Translation2d(0, 0), 0, false, true);
    }
    
    @Override
    public void execute() {}

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return true;    
    }
}
