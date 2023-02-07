// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.KOPSubsystem;

public class DriveJoystickCmd extends CommandBase {
  private final KOPSubsystem MotorSubsystem;
  private final Supplier<Double> SpeedFunction, TurnFunction;
  /** Creates a new DriverJoystickCmd. */
  public DriveJoystickCmd(KOPSubsystem subsystem, Supplier<Double> SpeedFunction, Supplier<Double> TurnFunction) {
    MotorSubsystem = subsystem;
    this.SpeedFunction = SpeedFunction;
    this.TurnFunction = TurnFunction;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(MotorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double Speed = SpeedFunction.get();
    double Turn = TurnFunction.get() * 0.7;

    double leftSpeed = Speed - Turn;
    double rightSpeed = Speed + Turn;

    MotorSubsystem.move(rightSpeed, leftSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    MotorSubsystem.stopModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
