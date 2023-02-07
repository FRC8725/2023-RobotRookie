package frc.robot.subsystems;

import frc.robot.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveMotorModule {
    private CANSparkMax NEOMotor;

    public DriveMotorModule(int Motor_Port, boolean Direction) {
        NEOMotor = new CANSparkMax(Motor_Port, MotorType.kBrushless);
        NEOMotor.setInverted(Direction);
        NEOMotor.setIdleMode(IdleMode.kCoast);
        NEOMotor.setSmartCurrentLimit(30);
    }

    public void setDesiredState(Double speed) {
        NEOMotor.set(speed * Constants.DriverConstants.kSpeed);
        SmartDashboard.putNumber("speed", speed * Constants.DriverConstants.kSpeed);
    }

    public void stop() {
        NEOMotor.set(0);
    }
}
