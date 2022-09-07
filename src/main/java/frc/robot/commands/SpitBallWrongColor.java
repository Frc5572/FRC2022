package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.InnerMagazine;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;

public class SpitBallWrongColor extends CommandBase {
    Turret turret;
    InnerMagazine innerMag;
    Shooter shooter;

    public SpitBallWrongColor(Turret turret, InnerMagazine innerMag, Shooter shooter) {
        this.turret = turret;
        this.innerMag = innerMag;
        this.shooter = shooter;
        addRequirements(turret, innerMag, shooter);
    }

    @Override
    public void execute() {
        new SequentialCommandGroup(shooter.enable(), new Parraturret.turretRight(), new ParallelRaceGroup(new WaitUntilCommand(innerMag.magSense == false), innerMag.enable())))
    }

    // @Override
    // public boolean isFinished() {

    // }

    @Override
    public void end(boolean interrupted) {

    }

}
