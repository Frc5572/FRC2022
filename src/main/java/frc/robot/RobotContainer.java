package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.autos.LimelightAuto;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.Magazine;
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
    private final Joystick driver = new Joystick(0);
    private final Joystick operator = new Joystick(1);

    private final SendableChooser<String> autoChooser = new SendableChooser<>();

    private Command autoCommand;
    private static final String limelightAuto = "Limelight Auto";
    // private final Button shooterMotor = new Button(
    // () -> Math.abs(operator.getRawAxis(XboxController.Axis.kRightTrigger.value)) > .4);

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    /* Driver Buttons */
    private final JoystickButton zeroGyro =
        new JoystickButton(driver, XboxController.Button.kY.value);


    boolean fieldRelative;
    boolean openLoop;

    /* Subsystems */
    private final Swerve swerveDrive = new Swerve();
    private final Shooter shooter = new Shooter();
    private final Magazine magazine = new Magazine();
    private Vision vision = new Vision();
    private final Button shooterCom = new Button(
        () -> Math.abs(operator.getRawAxis(XboxController.Axis.kRightTrigger.value)) > .4)
            .whenPressed(new InstantCommand(shooter::enable, shooter).andThen(
                new WaitUntilCommand(shooter::atSetpoint),
                new InstantCommand(magazine::startMagazine, magazine)))
            .whenReleased(new InstantCommand(shooter::disable, shooter));



    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        autoChooser.addOption("Limelight Auto", limelightAuto);
        SmartDashboard.putData("Choose Auto: ", autoChooser);
        swerveDrive.setDefaultCommand(
            new TeleopSwerve(swerveDrive, vision, driver, translationAxis, strafeAxis, rotationAxis,
                Constants.Swerve.isFieldRelative, Constants.Swerve.isOpenLoop));
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
    }

    /**
     * Gets the user's selected autonomous command.
     *
     * @return Returns autonomous command selected.
     */
    public Command getAutonomousCommand() {

        if (autoChooser.getSelected() == "Limelight Auto") {
            System.out.println("Limelight Auto");
            autoCommand = new LimelightAuto(swerveDrive, vision);
        }
        return autoCommand;

    }
}
