package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.autos.P0;
import frc.robot.autos.P1_3B;
import frc.robot.autos.P1_5B;
import frc.robot.autos.P2_4B;
import frc.robot.autos.P_2B;
import frc.robot.commands.AlignTurret;
import frc.robot.commands.DefaultLEDs;
import frc.robot.commands.FeedShooter;
import frc.robot.commands.InnerMagIntake;
import frc.robot.commands.InsidePC;
import frc.robot.commands.OutsidePC;
import frc.robot.commands.ShooterRPM;
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.WheelsIn;
import frc.robot.commands.ZeroMotorsWaitCommand;
import frc.robot.modules.Vision;
import frc.robot.subsystems.InnerMagazine;
import frc.robot.subsystems.InsideClimber;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.OuterMagazine;
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
    private final CommandXboxController driver = new CommandXboxController(Constants.driverID);
    private final CommandXboxController operator = new CommandXboxController(Constants.operatorID);

    // Initialize AutoChooser Sendable
    private final SendableChooser<Command> autoChooser = new SendableChooser<>();

    // Field Relative and openLoop Variables
    boolean fieldRelative;
    boolean openLoop;


    /* Subsystems */

    private final Swerve swerveDrive = new Swerve();
    private final InnerMagazine innerMagazine = new InnerMagazine();
    private final OuterMagazine outerMagazine = new OuterMagazine();
    private final Intake intake = new Intake();
    private final Turret turret = new Turret();
    private Vision vision = new Vision();
    private final Shooter shooter = new Shooter();
    private final InsideClimber insideClimber;
    private final OutsideClimber outsideClimber;
    public PneumaticHub ph = new PneumaticHub();
    private LEDs leds = new LEDs(Constants.LEDConstants.PWMPort, Constants.LEDConstants.LEDCount);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        ph.enableCompressorAnalog(90, 120);
        insideClimber = new InsideClimber(ph);
        outsideClimber = new OutsideClimber(ph);
        // Default Swerve Command
        swerveDrive.setDefaultCommand(new TeleopSwerve(swerveDrive, driver,
            Constants.Swerve.isFieldRelative, Constants.Swerve.isOpenLoop));
        // Default Turret Command
        turret.setDefaultCommand(new AlignTurret(turret, vision));
        leds.setDefaultCommand(new DefaultLEDs(leds));
        // hood.setDefaultCommand(new PositionHood(hood, vision));
        SmartDashboard.putData("Choose Auto: ", autoChooser);
        autoChooser.setDefaultOption("Do Nothing", new ZeroMotorsWaitCommand(swerveDrive, 1));
        autoChooser.addOption("P0", new P0(swerveDrive));
        autoChooser.addOption("P_2B",
            new P_2B(swerveDrive, shooter, innerMagazine, outerMagazine, intake, turret, vision));
        autoChooser.addOption("P1_3B",
            new P1_3B(swerveDrive, shooter, innerMagazine, outerMagazine, intake, turret, vision));
        autoChooser.addOption("P1_5B",
            new P1_5B(swerveDrive, shooter, innerMagazine, outerMagazine, intake, turret, vision));
        autoChooser.addOption("P2_4B",
            new P2_4B(swerveDrive, shooter, innerMagazine, outerMagazine, intake, turret, vision));
        // Configure the button bindings
        configureButtonBindings();
        swerveDrive.zeroGyro();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses
     * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
     * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        Trigger magSensor = new Trigger(() -> this.innerMagazine.magSense.get());
        Trigger turretAligned =
            new Trigger(() -> this.vision.getTargetAligned() && this.turret.alignEnabled);
        // Turn default lights back to 0 with start button.
        operator.start().whileTrue((new InstantCommand(() -> leds.pattern = 0)));
        // Turn default lights to 2 with POV up (0)
        operator.povUp().whileTrue(new InstantCommand(() -> leds.pattern = 2));
        // LEDs are blue when ball is loaded
        magSensor.and(turretAligned.negate())
            .whileTrue(new StartEndCommand(() -> leds.setColor(Color.kBlue), () -> {
            }, leds));
        // LEDs are green when ball is loaded and locked on
        magSensor.and(turretAligned)
            .whileTrue(new StartEndCommand(() -> leds.setColor(Color.kGreen), () -> {
            }, leds));
        // LEDs are red when limelight aligned but ball not loaded
        magSensor.negate().and(turretAligned)
            .whileTrue(new StartEndCommand(() -> leds.setColor(Color.kRed), () -> {
            }, leds));
        /* Driver Buttons */
        // Reset Gyro on Driver Y pressed
        driver.y().whileTrue((new InstantCommand(() -> swerveDrive.zeroGyro())));
        // Turn Off Turret For Rest of Match on Driver X Pressed
        operator.x().whileTrue(new InstantCommand(() -> turret.alignEnabled = !turret.alignEnabled));

        /* Button Mappings for Climber Motors */
        // Extend the Outside climber arms
       driver.leftBumper().whileTrue(new StartEndCommand(() -> outsideClimber.engageMotors(),
                () -> outsideClimber.stopMotors(), outsideClimber));

                // Retract the Outside climber arms
        driver.leftTrigger(Constants.stickDeadband).whileTrue(new StartEndCommand(() -> outsideClimber.retractMotors(),
        () -> outsideClimber.stopMotors(), outsideClimber));

        // Extend the Inside climber arms
        driver.rightBumper().whileTrue(new StartEndCommand(() -> insideClimber.engageMotors(),
                () -> insideClimber.stopMotors(), insideClimber));

        // Retract the Inside climber arms
        driver.rightTrigger(Constants.stickDeadband).whileTrue(new StartEndCommand(() -> insideClimber.retractMotors(),
        () -> insideClimber.stopMotors(), insideClimber));



        // Inside Pneumatics Activate on drive
        driver.b().whileTrue(new OutsidePC(outsideClimber));
        // Outside Pneumatics Activate on driver
        driver.a().whileTrue(new InsidePC(insideClimber));
        driver.start().whileTrue(new InstantCommand(() -> insideClimber.enableClimbers())
                .andThen(new InstantCommand(() -> outsideClimber.enableClimbers()))
                .andThen(new InstantCommand(() -> turret.alignEnabled = false))
                // Turns default LEDS to 1
                .andThen(new InstantCommand(() -> leds.pattern = 1)));

        /* Operator Buttons */

        // Enable Shooter Tape Line setpoint right trigger
        operator.rightTrigger().whileTrue(new StartEndCommand(() -> turret.alignEnabled = true,
                () -> turret.alignEnabled = false))
            .whileTrue(new ShooterRPM(this.shooter, 2400 / 60))
            .whileTrue(new FeedShooter(innerMagazine, outerMagazine, shooter, intake))
            .whileTrue(new WheelsIn(swerveDrive));

        // Enable Shooter Safety Location setpoint left trigger
        operator.leftTrigger().whileTrue(new StartEndCommand(() -> turret.alignEnabled = true,
                () -> turret.alignEnabled = false))
            .whileTrue(new ShooterRPM(this.shooter, 3250 / 60)) // 15 ft
            .whileTrue(new FeedShooter(innerMagazine, outerMagazine, shooter, intake))
            .whileTrue
            (new WheelsIn(swerveDrive));

        // Enable Shooter Magazine Combo While Operator A Button Held
        operator.a().whileTrue(new StartEndCommand(() -> turret.alignEnabled = true,
                () -> turret.alignEnabled = false))
            .whileTrue(new ShooterRPM(this.shooter, this.vision))
            .whileTrue(new FeedShooter(innerMagazine, outerMagazine, shooter, intake))
            .whileTrue(new WheelsIn(swerveDrive));

        // Deploy Intake and Run Magazine While Operator B Held
        operator.b().whileTrue(new StartEndCommand(() -> {
                intake.intakeDeploy();
                outerMagazine.magazineUp();
            }, () -> {
                intake.intakeRetract();
                outerMagazine.magazineStop();
            }, intake, outerMagazine).alongWith(new InnerMagIntake(innerMagazine)));
        // Run hopper down with POV down (180))
        operator.povUp().whileTrue(new StartEndCommand(() -> {
            innerMagazine.magazineDown();
            outerMagazine.magazineDown();
        }, () -> {
            innerMagazine.magazineStop();
            outerMagazine.magazineStop();
        }));
        // Right Turret Move While Operator Right Bumper Held
        operator.rightBumper().whileTrue(
            new StartEndCommand(() -> turret.turretRight(), () -> turret.turretStop(), turret));

        // Left Turret Move While Operator Left Bumper Held
        operator.leftBumper().whileTrue(
            new StartEndCommand(() -> turret.turretLeft(), () -> turret.turretStop(), turret));

        // Spit ball command
        operator.y().whileTrue(new SequentialCommandGroup(new InstantCommand(() -> shooter.spinShooter()),
                new WaitCommand(.2), new InstantCommand(() -> {
                    innerMagazine.magazineUp();
                    outerMagazine.magazineUp();
                })))
            .whileFalse(new InstantCommand(() -> {
                shooter.stopShooter();
                innerMagazine.magazineStop();
                outerMagazine.magazineStop();
            }));

        // magSensor.debounce(1.5, DebounceType.kFalling).whenActive(new InstantCommand(() -> {
        // shooter.setSetpoint(1000 / 60);
        // shooter.enable();
        // }));
        // Print out distance
        // new AxisButton(operator, XboxController.Axis.kRightTrigger.value)
        // .whileHeld(new FunctionalCommand(() -> {
        // }, () -> System.out.println(String.valueOf(vision.getDistance())), inter -> {
        // }, () -> false));
    }

    /**
     * Gets the user's selected autonomous command.
     *
     * @return Returns autonomous command selected.
     */
    public Command getAutonomousCommand() {
        // return new P1_3B(swerveDrive, shooter, innerMagazine, outerMagazine, intake, turret,
        // vision);
        return autoChooser.getSelected();
    }
}
