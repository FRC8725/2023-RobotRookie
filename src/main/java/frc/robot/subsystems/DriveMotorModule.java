package frc.robot.subsystems;

import frc.robot.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveMotorModule {
    private final CANSparkMax NEOMotor;

    public DriveMotorModule(int port, boolean inverted) {
        this.NEOMotor = new CANSparkMax(port, MotorType.kBrushless);
        this.NEOMotor.setInverted(inverted);
        this.NEOMotor.setIdleMode(IdleMode.kCoast);
        this.NEOMotor.setSmartCurrentLimit(30);
        this.NEOMotor.getEncoder().setPositionConversionFactor(1.0 / 13.5);
        this.NEOMotor.getEncoder().setVelocityConversionFactor(1.0 / 13.5 / 20.0);
    }

    public void setDesiredState(double speed) {
        double realSpeed = speed * Constants.DriverConstants.kSpeed;
        this.NEOMotor.set(realSpeed);
        SmartDashboard.putNumber("speed", realSpeed);
    }

    public void stop() {
        this.NEOMotor.set(0);
    }

    public double getEncoderPos() {
        return this.NEOMotor.getEncoder().getPosition();
    }

    public double getEncoderVel() {
        return this.NEOMotor.getEncoder().getVelocity();
    }

    public void setVoltage(double voltage) {
        this.NEOMotor.setVoltage(voltage);
    }
}
