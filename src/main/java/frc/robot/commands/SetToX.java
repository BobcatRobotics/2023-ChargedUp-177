package frc.robot.commands;

import BobcatLib.Subsystems.Swerve.SimpleSwerve.SwerveDrive;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;

public class SetToX extends Command {
    private SwerveDrive drivetrain;
    
    public SetToX(SwerveDrive dt) {
        drivetrain = dt;
        addRequirements(dt);
    }

    @Override
    public void initialize() {
        drivetrain.drive(new Translation2d(0, 0), 0, false,drivetrain.getGyroYaw(),drivetrain.getPose());
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
