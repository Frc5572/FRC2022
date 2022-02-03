package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Turret;

/**
 * Defines left turret move.
 */
public class LeftTurretMove extends CommandBase {

    private Turret turret;

    /**turretMoves turret left. */
    public LeftTurretMove(Turret turret) {

        this.turret = turret;
        addRequirements(turret);
    }

    @Override
    public void execute() {
        this.turret.turretLeft();
    }

    @Override
    public void end(boolean interrupted) {
        this.turret.turretStop();
    }
}
