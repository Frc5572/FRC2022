package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.InnerMagazine;
import frc.robot.subsystems.Shooter;

public class MagSpit extends CommandBase {
    InnerMagazine innerMagazine;
    Shooter shooter;

    public MagSpit(InnerMagazine innerMagazine, Shooter shooter) {
        this.innerMagazine = innerMagazine;
        this.shooter = shooter;
        addRequirements(shooter);
        addRequirements(innerMagazine);
    }

    @Override
    public void execute() {
        this.innerMagazine.magazineUp();
        this.shooter.spinShooter();
    }

    @Override
    public void end(boolean interrupted) {
        this.innerMagazine.magazineStop();
        this.shooter.stopShooter();
    }
}
