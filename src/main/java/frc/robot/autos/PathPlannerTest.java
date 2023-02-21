package frc.robot.autos;

import java.util.List;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.DriveFollowPath;
import frc.robot.commands.SetToX;
import frc.robot.subsystems.Swerve;

public class PathPlannerTest extends SequentialCommandGroup {
    public PathPlannerTest() {
        List<PathPlannerTrajectory> pptList = PathPlanner.loadPathGroup("New Path", 1, 1);
        
        addCommands(
            Swerve.followTrajectoryCommand(pptList.get(0), true),
            new SetToX(RobotContainer.s_Swerve),
            new WaitCommand(2),
            Swerve.followTrajectoryCommand(pptList.get(1), false)
        );
    }
}
