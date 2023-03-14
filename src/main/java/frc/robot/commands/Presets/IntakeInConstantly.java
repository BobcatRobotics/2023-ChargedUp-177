package frc.robot.commands.Presets;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class IntakeInConstantly extends CommandBase {
    private Intake intake;

    public IntakeInConstantly(Intake i) {
        this.intake = i;
        addRequirements(i);
    }

    @Override
    public void initialize() {
        intake.runIntakeIn();
    }

    @Override
    public void execute() {
        intake.runIntakeIn();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}