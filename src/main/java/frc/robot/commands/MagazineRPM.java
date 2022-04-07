package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.InnerMagazine;
import frc.robot.subsystems.Shooter;

/**
 * Controls Magazines RPM based on Shooter's Setpoint
 */
public class MagazineRPM extends CommandBase {

    private Shooter shooter;
    private InnerMagazine innerMagazine;

    /**
     *
     * @param shooter shooter subsystem
     * @param innerMagazine inner magazine subsystem
     */
    public MagazineRPM(Shooter shooter, InnerMagazine innerMagazine) {
        this.shooter = shooter;
        this.innerMagazine = innerMagazine;
        addRequirements(innerMagazine);
    }

    @Override
    public void initialize() {
        updateSetpoint();
        this.innerMagazine.enable();
    }

    @Override
    public void execute() {
        updateSetpoint();
        double selSenVel = innerMagazine.innerMagazineMotor.getSelectedSensorVelocity(0);
        double rotPerSec = (double) selSenVel / Constants.ShooterPID.kUnitsPerRevolution * 10;
        System.out.println("MAG RPM: " + rotPerSec * 60);
    }

    @Override
    public void end(boolean interrupted) {
        innerMagazine.setSetpoint(Constants.InnerMagazinePID.kInnerMagazineTargetRPS);
        this.innerMagazine.disable();
    }

    private void updateSetpoint() {
        double shooterRPM = this.shooter.getSetpoint();
        if (shooterRPM >= (3300 / 60)) {
            this.innerMagazine.setSetpoint(750 / 60);
        } else if (shooterRPM >= (3100 / 60)) {
            this.innerMagazine.setSetpoint(850 / 60);
        } else if (shooterRPM >= (2900 / 60)) {
            this.innerMagazine.setSetpoint(1000 / 60);
        } else if (shooterRPM >= (2000 / 60)) {
            this.innerMagazine.setSetpoint(1100 / 60);
        } else {
            this.innerMagazine.setSetpoint(1500 / 60);
        }

        System.out.println("MAG SETPOINT: " + this.innerMagazine.getSetpoint());
    }

    public void intakeMag() {

    }
}
