package frc.robot.subsystems;

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
    this.gyro.calibrate();
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

  public void outputVolts(Double d1, Double d2) {
    this.leftMotor1.setVoltage(d1);
    this.leftMotor2.setVoltage(d1);
    this.rightMotor1.setVoltage(d2);
    this.rightMotor2.setVoltage(d2);
  }

  public Pose2d getPose() {
    return this.estimator.getEstimatedPosition();
  }

  public DifferentialDriveKinematics getKinematics() {
    return this.kinematics;
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(this.leftMotor1.getEncoderVel() * Units.inchesToMeters(6) * 2 * Math.PI, this.rightMotor1.getEncoderVel() * Units.inchesToMeters(6) * 2 * Math.PI);
  }

  public void resetOdometry(Pose2d pose2d) {
    this.estimator.resetPosition(this.gyro.getRotation2d(), 0, 0, pose2d);
  }

  @Override
  public void periodic() {
    this.estimator.update(new Rotation2d(Units.degreesToRadians(this.gyro.getAngle()) * -1), this.leftMotor1.getEncoderPos(), this.rightMotor1.getEncoderPos());
    this.field.setRobotPose(this.estimator.getEstimatedPosition());
    SmartDashboard.putData(this.field);
    SmartDashboard.putNumber("Angle", Units.degreesToRadians(this.gyro.getAngle()) * -1);
    SmartDashboard.putNumber("LeftEncoder", this.leftMotor1.getEncoderPos());
    SmartDashboard.putNumber("RightEncoder", this.rightMotor1.getEncoderPos());
  }
}

