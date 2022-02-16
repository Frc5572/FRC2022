package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.autos.LimelightAuto;
import frc.robot.autos.TestAuto;
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.ZeroMotorsWaitCommand;
import frc.robot.modules.Vision;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Magazine;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Turret;

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
    private static final String limelightAuto = "Limelight Auto";

    boolean fieldRelative;
    boolean openLoop;

    /* Subsystems */
    private final Shooter shooter = new Shooter();
    private final Swerve swerveDrive = new Swerve();
    private final Magazine magazine = new Magazine();
    private final Intake intake = new Intake();
    private final Turret turret = new Turret();
    private Vision vision = new Vision();
    private final Hood hood = new Hood(vision);
    // private final Climber climber = new Climber();

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        swerveDrive.zeroGyro();
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
        // new JoystickButton(operator, XboxController.Button.kB.value)
        // .whenPressed(new InstantCommand(shooter::enable, shooter).andThen(
        // new WaitUntilCommand(() -> shooter.atSetpoint()),
        // new InstantCommand(magazine::enable, magazine)))
        // .whenReleased(new InstantCommand(shooter::disable, shooter))
        // .whenReleased(new InstantCommand(magazine::disable, magazine));
        // new JoystickButton(driver, XboxController.Button.kY.value)
        // .whenPressed(new InstantCommand(() -> swerveDrive.zeroGyro()));
        // new JoystickButton(driver, XboxController.Button.kY.value)
        // .whileHeld(new InstantCommand(() -> hood.getServoPos()));

        // new JoystickButton(driver, XboxController.Button.kA.value)
        // .whileHeld(new FunctionalCommand(magazine::enable, () -> {
        // }, interrupted -> magazine.disable(), () -> magazine.magSense.get(), magazine));
        // new JoystickButton(driver, XboxController.Button.kX.value)
        // .whileHeld(new TeleopSwerve(swerveDrive, vision, driver,
        // Constants.Swerve.isFieldRelative, Constants.Swerve.isOpenLoop, true));


        // new JoystickButton(driver, XboxController.Button.kA.value)
        // .whenPressed(new InstantCommand(() -> hood.hoodServo.setPosition(0)));
        // new JoystickButton(driver, XboxController.Button.kB.value)
        // .whenPressed(new InstantCommand(() -> hood.hoodServo.setPosition(1)));
        new JoystickButton(driver, XboxController.Button.kY.value)
            .whenPressed(new InstantCommand(() -> hood.hoodServo.setPosition(200)));
        new JoystickButton(driver, XboxController.Button.kX.value).whenPressed(
            new ParallelRaceGroup(new InstantCommand(() -> hood.hoodServo.setPosition(.5)),
                new InstantCommand(() -> System.out.println(hood.hoodServo.getPosition()))));
        // new JoystickButton(driver, XboxController.Button.kY.value)
        // .whenPressed(new InstantCommand(() -> swerveDrive.zeroGyro()));
        // new JoystickButton(driver, XboxController.Button.kA.value)
        // .whileHeld(new FunctionalCommand(magazine::enable, () -> {
        // }, interrupted -> magazine.disable(), () -> magazine.magSense.get(), magazine));
        // new JoystickButton(driver, XboxController.Button.kX.value)
        // .whileHeld(new TeleopSwerve(swerveDrive, vision, driver,
        // Constants.Swerve.isFieldRelative, Constants.Swerve.isOpenLoop, true));
        // // new JoystickButton(driver, XboxController.Button.kRightBumper.value)
        // // .whileHeld(new RightTurretMove(turret));
        // // new JoystickButton(driver, XboxController.Button.kLeftBumper.value)
        // // .whileHeld(new LeftTurretMove(turret));

        // new Button(() -> Math.abs(driver.getRawAxis(XboxController.Axis.kRightTrigger.value)) >
        // .4)
        // .whileHeld(new StartEndCommand(intake::in, intake::stop, intake));
        // new JoystickButton(driver, XboxController.Button.kLeftBumper.value)
        // .whileHeld(new LeftTurretMove(turret));
    }

    /**
     * Gets the user's selected autonomous command.
     *
     * @return Returns autonomous command selected.
     */
    public Command getAutonomousCommand() {
        // return autoChooser.getSelected();
        return new TestAuto(swerveDrive);
    }
}
