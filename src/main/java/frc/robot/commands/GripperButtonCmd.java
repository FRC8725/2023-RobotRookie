// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.GripperSubsystem;
import frc.robot.Constants;

import com.ctre.phoenix.led.ColorFlowAnimation.Direction;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class GripperButtonCmd extends CommandBase {
  private final GripperSubsystem GripperSubsystem;
  private boolean Direction;
  /** Creates a new JoystickButtonCmd. */
  public GripperButtonCmd(GripperSubsystem subsystem, Boolean direction) {
    GripperSubsystem = subsystem;
    Direction = direction;

    addRequirements(GripperSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    GripperSubsystem.grip(Direction ? Constants.GripperConstants.kGripperSpeed : -Constants.GripperConstants.kGripperSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    GripperSubsystem.stopModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
