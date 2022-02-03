package frc.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.autos.LimelightAuto;
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.ZeroMotorsWaitCommand;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    private final XboxController driver = new XboxController(Constants.driverID);
    private final XboxController operator = new XboxController(Constants.operatorID);

    private final SendableChooser<Command> autoChooser = new SendableChooser<>();

    // private final Button shooterMotor = new Button(
    // () -> Math.abs(operator.getRawAxis(XboxController.Axis.kRightTrigger.value)) > .4);
    private final Shooter shooter = new Shooter();
    DigitalInput magSensor = new DigitalInput(Constants.magazineSensor);



    private Vision vision = new Vision();

    boolean fieldRelative;
    boolean openLoop;

    /* Subsystems */
    private final Swerve swerveDrive = new Swerve();

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        SmartDashboard.putData("Choose Auto: ", autoChooser);
        autoChooser.setDefaultOption("Do Nothing", new ZeroMotorsWaitCommand(swerveDrive, 1));
        autoChooser.addOption("Limelight Auto", new LimelightAuto(swerveDrive, vision));
        swerveDrive.setDefaultCommand(new TeleopSwerve(swerveDrive, vision, driver,
            Constants.Swerve.isFieldRelative, Constants.Swerve.isOpenLoop, false));
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
        new JoystickButton(driver, XboxController.Button.kY.value)
            .whenPressed(new InstantCommand(() -> swerveDrive.zeroGyro()));
        new JoystickButton(driver, XboxController.Button.kX.value)
            .whileHeld(new TeleopSwerve(swerveDrive, vision, driver,
                Constants.Swerve.isFieldRelative, Constants.Swerve.isOpenLoop, true));
        // shooterMotor.whenHeld(new ShooterRev(shooter));

    }

    /**
     * Gets the user's selected autonomous command.
     *
     * @return Returns autonomous command selected.
     */
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
