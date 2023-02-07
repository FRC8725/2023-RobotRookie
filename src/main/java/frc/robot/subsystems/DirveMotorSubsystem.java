// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.security.spec.KeySpec;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.DriveMotorModule;
import frc.robot.robotMap;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DirveMotorSubsystem extends SubsystemBase {
  /** Creates a new DirverMotorSubsystem. */
  private DriveMotorModule RightMotor1;
  private DriveMotorModule RightMotor2;
  private DriveMotorModule LeftMotor1;
  private DriveMotorModule LeftMotor2;
  public DirveMotorSubsystem() {
    RightMotor1 = new DriveMotorModule(robotMap.kRightMotorPort1, false);
    RightMotor2 = new DriveMotorModule(robotMap.kRightMotorPort2, false);
    LeftMotor1 = new DriveMotorModule(robotMap.kLeftMotorPort1, true);
    LeftMotor2 = new DriveMotorModule(robotMap.kLeftMotorPort2, true);
  }

  public void move(Double rightSpeed, Double leftSpeed) {
    RightMotor1.setDesiredState(rightSpeed);
    RightMotor2.setDesiredState(rightSpeed);
    LeftMotor1.setDesiredState(leftSpeed);
    LeftMotor2.setDesiredState(leftSpeed);
  }

  public void stopModules() {
    RightMotor1.stop();
    RightMotor2.stop();
    LeftMotor1.stop();
    LeftMotor2.stop();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
