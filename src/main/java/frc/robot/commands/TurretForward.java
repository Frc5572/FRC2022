package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Turret;

/**
 * Command for aligning turret in TeleOp
 */

public class TurretForward extends CommandBase {
    Turret turret;
    int rotationsAtFront = Constants.TurretConstants.rotationsAtFront;
    double maxPower = Constants.TurretConstants.maxPower;
    double cancoderTolerance = Constants.TurretConstants.cancoderTolerance;

    /**
     *
     * @param turret turret subsystem
     */
    public TurretForward(Turret turret) {
        this.turret = turret;
        addRequirements(turret);
    }

    @Override
    public void initialize() {
        this.turret.alignEnabled = false;
    }

    @Override
    public void execute() {
        double power = 0;
        double cancoderError = this.turret.turretError();
        if (this.turret.rotations > rotationsAtFront) {
            power = -maxPower;
        } else if (this.turret.rotations < rotationsAtFront) {
            power = maxPower;
        } else if (this.turret.rotations == rotationsAtFront
            && Math.abs(cancoderError) > cancoderTolerance) {
            power = (cancoderError / 750);
        } else {
            power = 0;
        }
        this.turret.turretSet(power);
    }

    @Override
    public boolean isFinished() {
        double cancoderError = this.turret.turretError();
        return Math.abs(cancoderError) < cancoderTolerance;
    }
}
