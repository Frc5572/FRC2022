package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.modules.Limelight;
import frc.robot.modules.Limelight.VisionState;
import frc.robot.subsystems.Turret;

/**
 * Command for aligning turret in TeleOp
 */

public class AlignTurret extends CommandBase {
    Turret turret;
    Limelight limelight;

    /**
     *
     * @param turret turret subsystem
     * @param limelight limelight subsystem
     */
    public AlignTurret(Turret turret, Limelight limelight) {
        this.turret = turret;
        this.limelight = limelight;
        addRequirements(turret);
    }

    @Override
    public void execute() {
        if (this.turret.alignEnabled && limelight.getTargetFound()) {
            VisionState state = limelight.getState();
            double calculated = (state.xOffset / 125) * 3;
            calculated = (Math.abs(calculated) <= Constants.VisionConstants.deadPocket) ? 0
                : (calculated >= .3) ? .3 : calculated;

            turret.turretSet(calculated);
        } else {
            this.turret.turretSet(0);
        }
        this.limelight.setLEDMode(this.turret.alignEnabled);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
