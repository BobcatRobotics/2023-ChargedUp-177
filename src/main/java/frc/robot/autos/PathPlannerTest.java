package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveFollowPath;

public class PathPlannerTest extends SequentialCommandGroup {
    public PathPlannerTest() {
        addCommands(
            new DriveFollowPath("New Path", 1, 1)
        );
    }
}
