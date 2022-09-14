package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.InnerMagazine;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;

public class TurretSpitBall extends CommandBase {

    Turret turret;
    InnerMagazine innerMag;
    Shooter shooter;

    public TurretSpitBall(Turret turret, InnerMagazine innerMag, Shooter shooter) {
        this.turret = turret;
        this.innerMag = innerMag;
        this.shooter = shooter;
        addRequirements(turret, innerMag, shooter);
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

    }

}
