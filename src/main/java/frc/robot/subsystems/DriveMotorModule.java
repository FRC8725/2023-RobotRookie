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
        this.setupMotor(inverted);
    }

    private void setupMotor(boolean inverted) {
        this.NEOMotor.setInverted(inverted);
        this.NEOMotor.setIdleMode(IdleMode.kCoast);
        this.NEOMotor.setSmartCurrentLimit(Constants.Motor.SMART_CURRENT_LIMIT);
        this.NEOMotor.getEncoder().setPositionConversionFactor(Constants.Motor.POS_FACTOR);
        this.NEOMotor.getEncoder().setVelocityConversionFactor(Constants.Motor.VEL_FACTOR);
    }

    public void setDesiredState(double speed) {
        double realSpeed = speed * Constants.Drive.kSpeed;
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
