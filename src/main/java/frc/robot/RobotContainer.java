package frc.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    private final Joystick driver = new Joystick(0);
    private final Joystick operator = new Joystick(1);

    // private final Button shooterMotor = new Button(
    // () -> Math.abs(operator.getRawAxis(XboxController.Axis.kRightTrigger.value)) > .4);
    private final Shooter shooter = new Shooter();
    DigitalInput magSensor = new DigitalInput(Constants.magazineSensor);



    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    /* Driver Buttons */
    private final JoystickButton zeroGyro =
        new JoystickButton(driver, XboxController.Button.kY.value);
    // private final JoystickButton moveMotorNew = new JoystickButton(driver,
    // XboxController.Button.kA.value);
    // private final JoystickButton alignSwerve = new JoystickButton(driver,
    // XboxController.Button.kX.value);

    boolean fieldRelative;
    boolean openLoop;

    /* Subsystems */
    private final Swerve swerveDrive = new Swerve();

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        swerveDrive
            .setDefaultCommand(new TeleopSwerve(swerveDrive, driver, translationAxis, strafeAxis,
                rotationAxis, Constants.Swerve.isFieldRelative, Constants.Swerve.isOpenLoop));
        // Configure the button bindings
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses
     * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
     * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */
        zeroGyro.whenPressed(new InstantCommand(() -> swerveDrive.zeroGyro()));
        // shooterMotor.whenHeld(new ShooterRev(shooter));
    }
}
