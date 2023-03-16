package frc.robot.commands.Autos.AutoPresets;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Presets.SetArm;
import frc.robot.commands.Presets.SetElevator;
import frc.robot.commands.Presets.SetWrist;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Wrist;

// ONLY FOR SCORECUBEHIGH DURING AUTOS
public class ScoreCubeHighAutos extends SequentialCommandGroup {
    public ScoreCubeHighAutos(Elevator e, Arm a, Intake i, Wrist w) {
        addCommands(//will not work because we need to create the set elevator and arm commands
            new SetArm(a,1),//set arm to pos 1      
            Commands.parallel(new SetElevator(e,2),new SetArm(a,2))
        );
    }
} 
