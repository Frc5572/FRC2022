package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Turret;

/**
 * Command for aligning turret in TeleOp
 */

public class TurretLeft extends CommandBase {
    Turret turret;
    Timer timer;

    /**
     *
     * @param turret turret subsystem
     * @param vision vision subsystem
     */
    public TurretLeft(Turret turret) {
        this.turret = turret;
        Timer timer = new Timer();
        addRequirements(turret);
    }

    @Override
    public void execute() {
        turret.turretLeft();
    }

    @Override
    public boolean isFinished() {
        return timer.get() >= .25;
    }
}
