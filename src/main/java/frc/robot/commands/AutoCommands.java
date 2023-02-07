package frc.robot.commands;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPRamseteCommand;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.KOPSubsystem;

import java.util.List;

public class AutoCommands extends SequentialCommandGroup {
    private final KOPSubsystem subsystem;

    public AutoCommands(KOPSubsystem subsystem) {
        this.subsystem = subsystem;
        boolean isFirstPath = true;
        List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("Test", new PathConstraints(Constants.Auto.kMaxSpeedMetersPerSecond, Constants.Auto.kMaxAccelerationMetersPerSecondSquared));
        PathPlannerTrajectory firstTrajectory = pathGroup.get(0);
        InstantCommand command = this.getResetCommand(isFirstPath, firstTrajectory);
        PPRamseteCommand command1 = new PPRamseteCommand(firstTrajectory, subsystem::getPose, new RamseteController(), 
          new SimpleMotorFeedforward(0, 0, 0),
          subsystem.getKinematics(),
          subsystem::getWheelSpeeds,
          new PIDController(0.33, 0.5, 0),
          new PIDController(0.33, 0.5, 0),
          subsystem::outputVolts, true, subsystem);
          
        this.addCommands(command, command1);
        this.addRequirements(subsystem);
    }

    private InstantCommand getResetCommand(boolean isFirstPath, PathPlannerTrajectory trajectory) {
        return new InstantCommand(() -> {
            if (isFirstPath) {
                this.subsystem.resetOdometry(trajectory.getInitialPose());
            }
        });
    }
}
