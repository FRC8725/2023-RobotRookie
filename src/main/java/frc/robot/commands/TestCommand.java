package frc.robot.commands;

import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.KOPSubsystem;

public class TestCommand extends CommandBase {
    private final KOPSubsystem subsystem;

    public TestCommand(KOPSubsystem subsystem) {
        this.addRequirements(subsystem);
        this.subsystem = subsystem;
    }

    @Override
    public void initialize() {
        // TODO Auto-generated method stub
    }

    @Override
    public void execute() {
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        this.subsystem.stopModules();
    }
}
