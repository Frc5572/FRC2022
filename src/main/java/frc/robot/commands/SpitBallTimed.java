package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.InnerMagazine;
import frc.robot.subsystems.Shooter;

/**
 * Command for aligning turret in TeleOp
 */

public class SpitBallTimed extends WaitCommand {
    double time;

    /**
     *
     * @param turret turret subsystem
     * @param vision vision subsystem
     */
    public SpitBallTimed(InnerMagazine innerMag, Shooter shooter, double time) {
        this.time = time;
    }

    @Override
    public void execute() {
        if (this.turret.alignEnabled && vision.getTargetFound()) {
            this.turret.turretSet(vision.getAimValue());
            // this.turret.turretBrakeMode(false);
        } else {
            this.turret.turretSet(0);
            // this.turret.turretBrakeMode(true);
        }
        this.vision.setLEDMode(this.turret.alignEnabled);
        this.vision.setCameraMode(!this.turret.alignEnabled);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
