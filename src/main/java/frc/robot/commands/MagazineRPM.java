package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.InnerMagazine;
import frc.robot.subsystems.OuterMagazine;
import frc.robot.subsystems.Shooter;

/**
 * Controls Magazines RPM based on Shooter's Setpoint
 */
public class MagazineRPM extends CommandBase {

    private Shooter shooter;
    private InnerMagazine innerMagazine;
    private OuterMagazine outerMagazine;

    /**
     *
     * @param shooter shooter subsystem
     * @param magazine magazine subsystem
     */
    public MagazineRPM(Shooter shooter, InnerMagazine innerMagazine, OuterMagazine outerMagazine) {
        this.shooter = shooter;
        this.innerMagazine = innerMagazine;
        this.outerMagazine = outerMagazine;
        addRequirements(innerMagazine, outerMagazine);
    }

    @Override
    public void initialize() {
        updateSetpoint();
        // this.magazine.enable();
    }

    @Override
    public void execute() {
        updateSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        // magazine.setSetpoint(Constants.MagazinePID.kMagazineTargetRPS);
        // this.magazine.disable();
    }

    private void updateSetpoint() {
        double shooterRPM = this.shooter.getSetpoint();
        // if (shooterRPM >= (5800 / 60)) {
        // this.magazine.setSetpoint(1000 / 60);
        // } else {
        // this.magazine.setSetpoint(4000 / 60);
        // }
    }

    public void intakeMag() {

    }
}
