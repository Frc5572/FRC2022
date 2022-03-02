package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Magazine;
import frc.robot.subsystems.Shooter;

/**
 * Controls Shooter RPM based on
 */
public class MagazineRPM extends CommandBase {

    private Shooter shooter;
    private Magazine magazine;

    /**
     *
     * @param shooter shooter subsystem
     * @param magazine vision subsystem
     */
    public MagazineRPM(Shooter shooter, Magazine magazine) {
        this.shooter = shooter;
        this.magazine = magazine;
        addRequirements(magazine);
    }

    @Override
    public void initialize() {
        updateSetpoint();
        this.magazine.enable();
    }

    @Override
    public void execute() {
        updateSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        this.shooter.disable();
    }

    private void updateSetpoint() {
        double shooterRPM = this.shooter.getSetpoint();
        if (shooterRPM >= (5800 / 60)) {
            this.magazine.setSetpoint(1000 / 60);
        } else {
            this.magazine.setSetpoint(2000 / 60);
        }
    }
}
