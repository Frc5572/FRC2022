package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.subsystems.InnerMagazine;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;

public class TurretSpitBall extends SequentialCommandGroup {

    Turret turret;
    InnerMagazine innerMag;
    Shooter shooter;
    Timer timer = new Timer();

    public TurretSpitBall(Turret turret, InnerMagazine innerMag, Shooter shooter) {
        addCommands(
            // new TurretLeft(turret),
            new StartEndCommand(() -> {
                innerMag.magazineUp();
                SmartDashboard.putString("Spitting:", "Spitting");
                shooter.spinShooter();
            }, () -> {
                innerMag.magazineStop();
                SmartDashboard.putString("Not Spitting:", "Not Spitting");
                shooter.stopShooter();
            }, shooter, innerMag).withTimeout(2), new PrintCommand("none"), new InstantCommand(() -> endCommand()));
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
        SmartDashboard.putString("Running?", "Running");
        this.turret = turret;
        this.innerMag = innerMag;
        this.shooter = shooter;
        addRequirements(turret, innerMag, shooter);
        timer.start();
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


    private void endCommand() {
        timer.stop();
        turret.turretStop();
        shooter.disable();
        innerMag.disable();
    }

}