// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPRamseteCommand;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class KOPSubsystem extends SubsystemBase {
  private final DriveMotorModule rightMotor1;
  private final DriveMotorModule rightMotor2;
  private final DriveMotorModule leftMotor1;
  private final DriveMotorModule leftMotor2;
  private final ADXRS450_Gyro gyro;
  private final DifferentialDriveKinematics kinematics;
  private final DifferentialDrivePoseEstimator estimator;
  private final Field2d field = new Field2d();

  public KOPSubsystem() {
    this.rightMotor1 = new DriveMotorModule(RobotMap.RIGHT_MOTOR_1_PORT, false);
    this.rightMotor2 = new DriveMotorModule(RobotMap.RIGHT_MOTOR_2_PORT, false);
    this.leftMotor1 = new DriveMotorModule(RobotMap.LEFT_MOTOR_1_PORT, true);
    this.leftMotor2 = new DriveMotorModule(RobotMap.LEFT_MOTOR_2_PORT, true);
    this.gyro = new ADXRS450_Gyro(SPI.Port.kOnboardCS0);
    this.kinematics = new DifferentialDriveKinematics(0.55);
    this.estimator = new DifferentialDrivePoseEstimator(this.kinematics, new Rotation2d(Units.degreesToRadians(gyro.getAngle())), 0, 0, new Pose2d(0, 0, new Rotation2d(0)));
  }

  public void move(double rightSpeed, double leftSpeed) {
    this.rightMotor1.setDesiredState(rightSpeed);
    this.rightMotor2.setDesiredState(rightSpeed);
    this.leftMotor1.setDesiredState(leftSpeed);
    this.leftMotor2.setDesiredState(leftSpeed);
  }

  public void stopModules() {
    this.rightMotor1.stop();
    this.rightMotor2.stop();
    this.leftMotor1.stop();
    this.leftMotor2.stop();
  }

  public Command followTrajectoryCommand(PathPlannerTrajectory trajectory, boolean isFirstPath) {
    return new SequentialCommandGroup(
            new InstantCommand(() -> {
              if (isFirstPath){
                this.resetOdometry(trajectory.getInitialPose());
              }
            }),

            new PPRamseteCommand(
                    trajectory,
                    this::getPose, // Pose supplier
                    new RamseteController(),
                    new SimpleMotorFeedforward(0.5, 0.5, 0.5),
                    this.kinematics, // DifferentialDriveKinematics
                    this::getWheelSpeeds, // DifferentialDriveWheelSpeeds supplier
                    new PIDController(0.5, 0, 0), // Left controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
                    new PIDController(0.5, 0, 0), // Right controller (usually the same values as left controller)
                    this::outputVolts, // Voltage biConsumer
                    true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
                    this // Requires this drive subsystem
            )
    );
  }

  private void outputVolts(Double d1, Double d2) {}

  private Pose2d getPose() {
    return this.estimator.getEstimatedPosition();
  }

  private DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(this.leftMotor1.getEncoderVel(), this.rightMotor1.getEncoderVel());
  }

  private void resetOdometry(Pose2d pose2d) {
    this.estimator.resetPosition(this.gyro.getRotation2d(), 0, 0, pose2d);
  }

  @Override
  public void periodic() {
    this.estimator.update(new Rotation2d(this.gyro.getAngle()), this.leftMotor1.getEncoderPos(), this.rightMotor1.getEncoderPos());
    this.field.setRobotPose(this.estimator.getEstimatedPosition());
    SmartDashboard.putData(this.field);
  }
}
