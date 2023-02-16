// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.RobotMap;

public class GripperSubsystem extends SubsystemBase {
  private static CANSparkMax Gripmotor;

  public GripperSubsystem() {
    Gripmotor = new CANSparkMax(RobotMap.Grip_MOTOR_PORT, MotorType.kBrushless);
  }

  public void grip(Double speed) {
    Gripmotor.set(speed);
    SmartDashboard.putNumber("GripperSpeed", speed);
  }

  public void gripperArm(Double speed) {

  }

  public void stopModules() {
    Gripmotor.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
