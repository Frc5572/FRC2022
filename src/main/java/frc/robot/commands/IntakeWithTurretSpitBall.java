package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.modules.ColorSensor;
import frc.robot.subsystems.InnerMagazine;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.OuterMagazine;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;

public class IntakeWithTurretSpitBall extends SequentialCommandGroup {

    Turret turret;
    InnerMagazine innerMagazine;
    Shooter shooter;
    Timer timer = new Timer();
    boolean shouldSpit = false;

    public IntakeWithTurretSpitBall(Turret turret, InnerMagazine innerMagazine, Shooter shooter,
        Intake intake, OuterMagazine outerMagazine, ColorSensor colorSensor) {
        addCommands(
            // new TurretLeft(turret),
            new StartEndCommand(() -> {
                innerMagazine.magazineUp();
                SmartDashboard.putString("Spitting:", "Spitting");
                shooter.spinShooter();
            }, () -> {
                innerMagazine.magazineStop();
                SmartDashboard.putString("Not Spitting:", "Not Spitting");
                shooter.stopShooter();
            }, shooter, innerMagazine), new PrintCommand("none"), new StartEndCommand(() -> {
                intake.intakeDeploy();
                outerMagazine.magazineUp();
                SmartDashboard.putString("Should it spit?", "" + colorSensor.shouldSpit());
            }, () -> {
                intake.intakeRetract();
                outerMagazine.magazineStop();
            }, intake, outerMagazine).alongWith(new InnerMagIntake(innerMagazine)),
            new InstantCommand(() -> shouldSpit = colorSensor.shouldSpit()),
            new ConditionalCommand(new ParallelCommandGroup(
                new TurretSpitBall(turret, innerMagazine, shooter), new InstantCommand(() -> {
                    SmartDashboard.putString("Should Spit:", "True");
                })), new InstantCommand(() -> {
                    SmartDashboard.putString("Should Spit:", "False");
                }), () -> shouldSpit));
        // new FunctionalCommand(() -> {
        // }, () -> {

        // }, interrupt -> {

        // }, new WaitCommand(.25)::isFinished, innerMag, shooter)
        // new InstantCommand(() -> SmartDashboard.putString("Turret Righting:", "Turret RIght")),
        // new TurretRight(turret), new InstantCommand(() -> {
        // turret.turretStop();
        // shooter.disable();
        // innerMag.disable();
        // })

        // super(new TurretLeft(turret),
        // new InstantCommand(() -> SmartDashboard.putString("Turret Left", "Turret is Lefting")),
        // new FunctionalCommand(() -> {
        // }, () -> {
        // innerMag.magazineUp();
        // SmartDashboard.putString("Spitting:", "Spitting");
        // shooter.spinShooter();
        // }, interrupt -> {
        // innerMag.magazineStop();
        // SmartDashboard.putString("Not Spitting:", "Not Spitting");
        // shooter.stopShooter();
        // }, () -> true, innerMag, shooter),
        // new InstantCommand(() -> SmartDashboard.putString("Turret Righting:", "Turret RIght")),
        // new TurretRight(turret), new InstantCommand(() -> {
        // turret.turretStop();
        // shooter.disable();
        // innerMag.disable();
        // }));
        // SmartDashboard.putString("Running?", "Running");
        // this.turret = turret;
        // this.innerMag = innerMag;
        // this.shooter = shooter;
        // addRequirements(turret, innerMag, shooter);
        // timer.start();
    }

    /*
     * FunctionalCommand turnTurretRight = new FunctionalCommand(() -> { turretSpitWait = new
     * WaitCommand(.25); }, () -> { turret.turretRight(); }, interrupt -> turret.turretStop(), new
     * WaitCommand(.25)::isFinished, turret);
     * 
     * FunctionalCommand turnTurretLeft = new FunctionalCommand(() -> { // turretSpitWait = new
     * WaitCommand(.25); }, () -> { turret.turretLeft(); }, interrupt -> turret.turretStop(), new
     * WaitCommand(.25)::isFinished, turret);
     * 
     * FunctionalCommand innerMagRun = new FunctionalCommand(() -> { // innerMagSpitWait = new
     * WaitCommand(1); }, () -> { innerMagazine.enable(); }, interrupt -> { innerMagazine.disable();
     * }, new WaitCommand(1)::isFinished, innerMagazine);
     */


    @Override
    public void execute() {
        // timer.start();
        // if (timer.get() <= .25) {
        // SmartDashboard.putString("Right: ", "Turning Right:;::::");
        // turret.turretRight();
        // } else if (timer.get() >= .25 && timer.get() <= .5) {
        // SmartDashboard.putString("Left: ", "Turning Left:;::::");

        // turret.turretLeft();
        // }
        // SmartDashboard.putNumber("Timer: ", timer.get());

        // new InstantCommand( () -> {SmartDashboard.putString("Turret Left", "Turret is
        // Lefting")}),
        // timer.start();


        // new TurretLeft(turret);
        // SmartDashboard.putString("Turret Left", "Turret is Lefting");
        // timer.start();
        // new FunctionalCommand(() -> {
        // }, () -> {
        // innerMag.magazineUp();
        // SmartDashboard.putString("Spitting:", "Spitting");
        // shooter.spinShooter();
        // }, interrupt -> {
        // innerMag.magazineStop();
        // SmartDashboard.putString("Not Spitting:", "Not Spitting");
        // shooter.stopShooter();
        // }, () -> true, innerMag, shooter);
        // SmartDashboard.putString("Turret Righting:", "Turret RIght");
        // new TurretRight(turret);


    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();
        turret.turretStop();
        shooter.disable();
        innerMagazine.disable();
    }

}
