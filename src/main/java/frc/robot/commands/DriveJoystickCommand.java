// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.KOPSubsystem;

public class DriveJoystickCommand extends CommandBase {
    private final KOPSubsystem subsystem;
    private final Supplier<Double> speedFunction, turnFunction;
    /** Creates a new DriverJoystickCmd. */
    public DriveJoystickCommand(KOPSubsystem subsystem, Supplier<Double> speedFunction, Supplier<Double> turnFunction) {
        this.subsystem = subsystem;
        this.speedFunction = speedFunction;
        this.turnFunction = turnFunction;
        this.addRequirements(subsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double speed = speedFunction.get();
        double turn = turnFunction.get() * 0.7;
        double leftSpeed = speed - turn;
        double rightSpeed = speed + turn;
        subsystem.move(rightSpeed, leftSpeed);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        subsystem.stopModules();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
