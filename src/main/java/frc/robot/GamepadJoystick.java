package frc.robot;
import edu.wpi.first.wpilibj.Joystick;

public class GamepadJoystick extends Joystick {
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int DRIVER_Y_AXIS = 1;
    public static final int DRIVER_X_AXIS = 2;

    public GamepadJoystick(int port) {
        super(port);
    }
}
