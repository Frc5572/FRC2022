package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.modules.Vision;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterRoller;

/**
 * Controls Shooter RPM based on
 */
public class ShooterRPM extends CommandBase {

    private Shooter shooter;
    private ShooterRoller shooterRoller;
    private Vision vision;
    double curDisRPM = 0;
    double newDisRPM = 0;

    /**
     *
     * @param shooter shooter subsystem
     * @param shooterRoller shooter roller subsystem
     * @param vision vision subsystem
     */
    public ShooterRPM(Shooter shooter, ShooterRoller shooterRoller, Vision vision) {
        this.shooter = shooter;
        this.shooterRoller = shooterRoller;
        this.vision = vision;
        addRequirements(shooter, shooterRoller);
    }

    @Override
    public void initialize() {
        // System.out.println("STARTING SHOOTER");
        updateSetpoint();
        // System.out.println("Initial RPM: " + this.shooter.getSetpoint());
        this.shooter.enable();
    }

    @Override
    public void execute() {
        updateSetpoint();
        // System.out.println("SHOOTER SETPOINT: " + this.shooter.getSetpoint());
    }

    @Override
    public void end(boolean interrupted) {
        curDisRPM = 0;
        shooter.setSetpoint(curDisRPM);
        this.shooter.disable();
    }

    private void updateSetpoint() {
        double distance = this.vision.getDistance() / 12;
        newDisRPM =
            3.452380952381 * Math.pow(distance, 3) - 61.7857142857143 * Math.pow(distance, 2)
                + 402.6190476190476 * Math.pow(distance, 1) + 3000;
        // newDisRPM = 3.45238 * Math.pow(distance, 3) - 51.42857 * Math.pow(distance, 2)
        // + 289.40476 * Math.pow(distance, 1) + 3300;
        // newDisRPM = 3.45238 * Math.pow(distance, 3) - 56 * Math.pow(distance, 2)
        // + 345 * Math.pow(distance, 1) + 2975;
        if (Math.abs(curDisRPM - newDisRPM) >= 100) {
            if (newDisRPM >= 6000) {
                curDisRPM = 6000;
            } else if (newDisRPM <= 3500) {
                curDisRPM = 3500;
            } else {
                curDisRPM = newDisRPM;
            }
            this.shooter.setSetpoint(curDisRPM / 60);
            this.shooterRoller.setSetpoint(curDisRPM / 60);
        }
    }
}
