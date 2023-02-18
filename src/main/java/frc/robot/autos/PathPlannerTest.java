package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveFollowPath;
import frc.robot.Constants;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlanner;

import java.util.ArrayList;

public class PathPlannerTest extends SequentialCommandGroup {
    public PathPlannerTest() {
        ArrayList<PathPlannerTrajectory> pathPlannerTestPaths = PathPlanner.loadPathGroup(
            "New Path",
            Constants.AutoConstants.kMaxSpeedMetersPerSecond,
            Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared
        );


        addCommands(
            
        );
    }
}
