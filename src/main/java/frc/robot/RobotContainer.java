package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.autos.LimelightAuto;
import frc.robot.autos.P0;
import frc.robot.autos.P_2B;
import frc.robot.commands.AlignTurret;
import frc.robot.commands.InsidePC;
import frc.robot.commands.OutsidePC;
import frc.robot.commands.ShooterRPM;
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.ZeroMotorsWaitCommand;
import frc.robot.modules.Vision;
import frc.robot.subsystems.InsideClimber;
// import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Magazine;
import frc.robot.subsystems.OutsideClimber;
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
    private final Intake intake;
    private final Turret turret = new Turret();
    private Vision vision = new Vision();
    private final Shooter shooter = new Shooter(vision);
    // private final Hood hood = new Hood(vision);
    private final InsideClimber insideClimber;
    private final OutsideClimber outsideClimber;
    public PneumaticHub ph = new PneumaticHub();

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        ph.enableCompressorAnalog(100, 120);
        insideClimber = new InsideClimber(ph);
        outsideClimber = new OutsideClimber(ph);
        intake = new Intake(ph);
        turret.setDefaultCommand(new AlignTurret(turret, vision));
        // hood.setDefaultCommand(new PositionHood(hood, vision.getHoodValue()));
        // Adding AutoChooser Options
        SmartDashboard.putData("Choose Auto: ", autoChooser);
        autoChooser.setDefaultOption("Do Nothing", new ZeroMotorsWaitCommand(swerveDrive, 1));
        autoChooser.addOption("Limelight Auto", new LimelightAuto(swerveDrive, turret, vision));
        autoChooser.addOption("P0", new P0(swerveDrive));
        autoChooser.addOption("P_2B",
            new P_2B(swerveDrive, shooter, magazine, intake, turret, vision));
        // Default Swerve Command
        swerveDrive.setDefaultCommand(new TeleopSwerve(swerveDrive, driver,
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
        // Reset Gyro on Driver Y pressed
        new JoystickButton(driver, XboxController.Button.kY.value)
            .whenPressed(new InstantCommand(() -> swerveDrive.zeroGyro()));
        // Turn Off Turret For Rest of Match on Driver X Pressed
        new JoystickButton(driver, XboxController.Button.kX.value)
            .whenPressed(new InstantCommand(() -> turret.alignEnabled = !turret.alignEnabled));

        /* Button Mappings for Climber Motors */
        // Extend the Outside climber arms
        new JoystickButton(driver, XboxController.Button.kLeftBumper.value)
            .whileHeld(new StartEndCommand(() -> outsideClimber.engageMotors(),
                () -> outsideClimber.stopMotors(), outsideClimber));
        // Retract the Outside climber arms
        new Button(() -> Math.abs(driver.getRawAxis(XboxController.Axis.kLeftTrigger.value)) > .4)
            .whileHeld(new StartEndCommand(() -> outsideClimber.retractMotors(),
                () -> outsideClimber.stopMotors(), outsideClimber));
        // Extend the Inside climber arms
        new JoystickButton(driver, XboxController.Button.kRightBumper.value)
            .whileHeld(new StartEndCommand(() -> insideClimber.engageMotors(),
                () -> insideClimber.stopMotors(), insideClimber));
        // Retract the Inside climber arms
        new Button(() -> Math.abs(driver.getRawAxis(XboxController.Axis.kRightTrigger.value)) > .4)
            .whileHeld(new StartEndCommand(() -> insideClimber.retractMotors(),
                () -> insideClimber.stopMotors(), insideClimber));

        // Inside Pneumatics Activate On Operator
        new JoystickButton(driver, XboxController.Button.kA.value)
            .whenPressed(new InsidePC(insideClimber));
        // Outside Pneumatics Activate On Operator
        new JoystickButton(driver, XboxController.Button.kB.value)
            .whenPressed(new OutsidePC(outsideClimber));
        new JoystickButton(driver, XboxController.Button.kStart.value)
            .whenPressed(new InstantCommand(() -> insideClimber.enableClimbers())
                .andThen(new InstantCommand(() -> outsideClimber.enableClimbers())));

        // // Operator POV Up - INside Motors Out
        // new POVButton(driver, 0).whileHeld(new StartEndCommand(
        // () -> insideClimber.engageInsideMotors(), () -> insideClimber.stopInsideMotors()));
        // // Operator POV Down - Inside Motors In
        // new POVButton(driver, 90).whileHeld(new StartEndCommand(
        // () -> insideClimber.retractInsideMotors(), () -> insideClimber.stopInsideMotors()));
        // // Operator POV Right - Outside Motors In
        // new POVButton(driver, 180).whileHeld(new StartEndCommand(
        // () -> insideClimber.retractOutsideMotors(), () -> insideClimber.stopOutsideMotors()));
        // // Operator POV Left - Outside Motors OUt
        // new POVButton(driver, 270).whileHeld(new StartEndCommand(
        // () -> insideClimber.engageOutsideMotors(), () -> insideClimber.stopOutsideMotors()));

        /* Operator Buttons */

        // Enable Shooter Magazine Combo While Operator A Button Held
        new JoystickButton(operator, XboxController.Button.kA.value)
            .whileHeld(new ParallelCommandGroup(new ShooterRPM(shooter, vision),
                new SequentialCommandGroup(new WaitUntilCommand(() -> shooter.atSetpoint()),
                    new InstantCommand(magazine::enable, magazine))))
            .whenReleased(new InstantCommand(shooter::disable, shooter))
            .whenReleased(new InstantCommand(magazine::disable, magazine));
        // Deploy Intake and Run Magazine While Operator B Held
        new JoystickButton(operator, XboxController.Button.kB.value).whileHeld(
            new StartEndCommand(() -> intake.intakeDeploy(), () -> intake.intakeRetract(), intake));
        new JoystickButton(operator, XboxController.Button.kB.value)
            .whenPressed(new FunctionalCommand(magazine::enable, () -> {
            }, interrupted -> magazine.disable(), () -> magazine.magSense.get(), magazine))
            .whenReleased(new InstantCommand(magazine::disable, magazine));
        // Right Turret Move While Operator Right Bumper Held
        new JoystickButton(operator, XboxController.Button.kRightBumper.value).whileHeld(
            new StartEndCommand(() -> turret.turretRight(), () -> turret.turretStop(), turret));
        // Left Turret Move While Operator Left Bumper Held
        new JoystickButton(operator, XboxController.Button.kLeftBumper.value).whileHeld(
            new StartEndCommand(() -> turret.turretLeft(), () -> turret.turretStop(), turret));
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
