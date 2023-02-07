package frc.robot.commands;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.KOPSubsystem;

import java.util.List;

public class AutoCommands extends SequentialCommandGroup {
    public AutoCommands(KOPSubsystem subsystem) {
        List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("Test", new PathConstraints(Constants.AutoConstants.kMaxSpeedMetersPerSecond, Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared));
        PathPlannerTrajectory firstTrajectory = pathGroup.get(0);
        this.addRequirements(subsystem);
        this.addCommands(subsystem.followTrajectoryCommand(firstTrajectory, true));
    }
}
