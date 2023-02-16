package frc.robot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class GamepadJoystick extends Joystick {
    public GamepadJoystick(int port) {
        super(port);
    }
    
    public static final int kDriverControllerPort = 0;
    
    public static final int kDriverYAxis = 1;
    public static final int kDriverXAxis = 2;

    public Trigger btn_x = new JoystickButton(this, 1);
    public Trigger btn_a = new JoystickButton(this, 2);
    public Trigger btn_b = new JoystickButton(this, 3);
    public Trigger btn_y = new JoystickButton(this, 4);

    public Trigger btn_left_up = new JoystickButton(this, 5);
    public Trigger btn_left_down = new JoystickButton(this, 7);

    public Trigger btn_right_up = new JoystickButton(this, 6);
    public Trigger btn_right_down = new JoystickButton(this, 8);

}
