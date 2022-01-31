package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Turret;

public class RightTurretMove extends CommandBase {

    private Turret turret;

    public RightTurretMove(Turret turret) {

        this.turret = turret;
        addRequirements(turret);
    }

    @Override
    public void execute() {
        this.turret.turretRight();
    }

    @Override
    public void end(boolean interrupted) {
        this.turret.turretStop();
    }
}
