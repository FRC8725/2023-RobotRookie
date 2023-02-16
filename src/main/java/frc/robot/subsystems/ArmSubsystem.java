// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.RobotMap;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {
  private static CANSparkMax GripperTurnMotor;
  private static CANSparkMax ArmMotor;
  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {
    GripperTurnMotor = new CANSparkMax(RobotMap.Grip_ARM_MOTOR_PORT, MotorType.kBrushless);
    ArmMotor = new CANSparkMax(RobotMap.ARM_MOTOR_PORT, MotorType.kBrushless);
  }

  public void GripperArmMove(Double speed) {
    GripperTurnMotor.set(speed);
    SmartDashboard.putNumber("GripperTurn", GripperTurnMotor.getEncoder().getPosition());
  }

  public void ArmMove(Double speed) {
    ArmMotor.set(speed);
    SmartDashboard.putNumber("Arm", ArmMotor.getEncoder().getPosition());
  }

  public void stopModules() {
    GripperTurnMotor.set(0);
    ArmMotor.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
