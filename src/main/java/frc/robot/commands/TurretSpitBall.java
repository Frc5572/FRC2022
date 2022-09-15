package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.InnerMagazine;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;

public class TurretSpitBall extends CommandBase {

    Turret turret;
    InnerMagazine innerMag;
    Shooter shooter;
    Timer timer = new Timer();

    public TurretSpitBall(Turret turret, InnerMagazine innerMag, Shooter shooter) {
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


    @Override
    public void execute() {
        timer.start();
        if (timer.get() <= .25) {
            turret.turretRight();
        } else if (timer.get() >= .25 && timer.get() <= .5) {
            turret.turretRight();
        }
    }

    @Override
    public boolean isFinished() {
        return timer.get() >= .5;
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();
        turret.turretStop();
        shooter.disable();
        innerMag.disable();
    }

}
