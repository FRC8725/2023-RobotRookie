package frc.robot;

import frc.robot.Constants.Operation;
import frc.robot.commands.AutoCommands;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.DriveJoystickCommand;
import frc.robot.subsystems.KOPSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems and commands are defined here...
    private final ExampleSubsystem exampleSubsystem = new ExampleSubsystem();
    private final KOPSubsystem driverSubsystem = new KOPSubsystem();
    private final GamepadJoystick driverJoystick = new GamepadJoystick(GamepadJoystick.DRIVER_CONTROLLER_PORT);
    // Replace with CommandPS4Controller or CommandJoystick if needed
    private final CommandXboxController driverController =
            new CommandXboxController(Operation.kDriverControllerPort);

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        // Configure the trigger bindings
        this.driverSubsystem.setDefaultCommand(new DriveJoystickCommand(this.driverSubsystem,
                () -> -this.driverJoystick.getRawAxis(GamepadJoystick.DRIVER_Y_AXIS),
                () -> this.driverJoystick.getRawAxis(GamepadJoystick.DRIVER_X_AXIS)));

        configureBindings();
    }

    /**
     * Use this method to define your trigger->command mappings. Triggers can be created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
     * predicate, or via the named factories in {@link
     * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
     * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
     * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
     * joysticks}.
     */

    private void configureBindings() {
        // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
        new Trigger(this.exampleSubsystem::exampleCondition)
                .onTrue(new ExampleCommand(this.exampleSubsystem));

        // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
        // cancelling on release.

        this.driverController.b().whileTrue(exampleSubsystem.exampleMethodCommand());
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return new AutoCommands(this.driverSubsystem);
    }
}
