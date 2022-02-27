package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.autos.LimelightAuto;
import frc.robot.autos.P0;
import frc.robot.autos.P3_4B;
import frc.robot.autos.P_2B;
import frc.robot.commands.InsidePC;
import frc.robot.commands.LeftTurretMove;
import frc.robot.commands.OutsidePC;
import frc.robot.commands.PositionHood;
import frc.robot.commands.RightTurretMove;
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.ZeroMotorsWaitCommand;
import frc.robot.modules.Vision;
import frc.robot.subsystems.Climber;
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

    // Initialize AutoChooser Sendable
    private final SendableChooser<Command> autoChooser = new SendableChooser<>();

    // Field Relative and openLoop Variables
    boolean fieldRelative;
    boolean openLoop;

    /* Subsystems */
    private final Swerve swerveDrive = new Swerve();
    private final Magazine magazine = new Magazine();
    private final Shooter shooter = new Shooter();
    private final Intake intake;
    private final Turret turret = new Turret();
    private Vision vision = new Vision();
    private final Hood hood = new Hood(vision);
    private final Climber climber;
    public PneumaticHub ph = new PneumaticHub();

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        ph.enableCompressorAnalog(100, 120);
        climber = new Climber(ph);
        intake = new Intake(ph);
        hood.setDefaultCommand(new PositionHood(hood, vision.getHoodValue()));
        // Adding AutoChooser Options
        SmartDashboard.putData("Choose Auto: ", autoChooser);
        autoChooser.setDefaultOption("Do Nothing", new ZeroMotorsWaitCommand(swerveDrive, 1));
        autoChooser.addOption("Limelight Auto", new LimelightAuto(swerveDrive, vision));
        autoChooser.addOption("P0", new P0(swerveDrive));
        autoChooser.addOption("P_2B",
            new P_2B(swerveDrive, shooter, magazine, intake, turret, vision));
        autoChooser.addOption("P3_4B", new P3_4B(swerveDrive));
        // Default Swerve Command
        swerveDrive.setDefaultCommand(new TeleopSwerve(swerveDrive, vision, driver,
            Constants.Swerve.isFieldRelative, Constants.Swerve.isOpenLoop, false));
        // Turret Default Command
        turret.setDefaultCommand(new FunctionalCommand(() -> {
        }, () -> turret.turretSet(vision.getTargetFound() ? vision.getAimValue() : 0), interupt -> {
        }, () -> false, turret));
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

        // Reset Gyro on Driver Y pressed
        new JoystickButton(driver, XboxController.Button.kY.value)
            .whenPressed(new InstantCommand(() -> swerveDrive.zeroGyro()));

        /* Operator Buttons */

        // Enable Shooter Magazine Combo While Operator A Button Held
        new JoystickButton(operator, XboxController.Button.kA.value)
            .whenPressed(new FunctionalCommand(magazine::enable, () -> {
            }, interrupted -> magazine.disable(), () -> magazine.magSense.get(), magazine))
            .whenReleased(new InstantCommand(magazine::disable, magazine));
        // Deploy Intake While Operator B Held
        new JoystickButton(operator, XboxController.Button.kB.value).whileHeld(
            new StartEndCommand(() -> intake.intakeDeploy(), () -> intake.intakeRetract(), intake));
        // Right Turret Move While Operator Right Bumper Held
        new JoystickButton(operator, XboxController.Button.kRightBumper.value)
            .whileHeld(new RightTurretMove(turret));
        // Left Turret Move While Operator Left Bumper Held
        new JoystickButton(operator, XboxController.Button.kLeftBumper.value)
            .whileHeld(new LeftTurretMove(turret));
        // Inside Pneumatics Activate On Operator
        new JoystickButton(driver, XboxController.Button.kX.value)
            .whenPressed(new InsidePC(climber));
        // Outside Pneumatics Activate On Operator
        new JoystickButton(driver, XboxController.Button.kY.value)
            .whenPressed(new OutsidePC(climber));

        /* POV Button Mappings for Climber Motors */

        // Operator POV Up - Outside Motors Out
        new POVButton(driver, 0).whileHeld(new StartEndCommand(() -> climber.engageOutsideMotors(),
            () -> climber.stopOutsideMotors()));
        // Operator POV Down - Outside Motors In
        new POVButton(driver, 180).whileHeld(new StartEndCommand(
            () -> climber.retractOutsideMotors(), () -> climber.stopOutsideMotors()));
        // Operator POV Right - Inside Motors Out
        new POVButton(driver, 90).whileHeld(new StartEndCommand(() -> climber.engageInsideMotors(),
            () -> climber.stopInsideMotors()));
        // Operator POV Left - Inside Motors In
        new POVButton(driver, 270).whileHeld(new StartEndCommand(
            () -> climber.retractInsideMotors(), () -> climber.stopInsideMotors()));
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
