package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.modules.Limelight;
import frc.robot.subsystems.Shooter;

/**
 * Controls Shooter RPM based on
 */
public class ShooterRPM extends CommandBase {

    private Shooter shooter;
    private Limelight limelight;
    double curDisRPM = 0;
    double newDisRPM = 0;

    /**
     *
     * @param shooter shooter subsystem
     * @param limelight limelight subsystem
     */
    public ShooterRPM(Shooter shooter, Limelight limelight) {
        this.shooter = shooter;
        this.limelight = limelight;
        addRequirements(shooter);
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
        double distance = this.limelight.getDistance() / 12;
        newDisRPM =
            3.452380952381 * Math.pow(distance, 3) - 61.7857142857143 * Math.pow(distance, 2)
                + 402.6190476190476 * Math.pow(distance, 1) + 3000;
        // newDisRPM = 3.45238 * Math.pow(distance, 3) - 51.42857 * Math.pow(distance, 2)
        // + 289.40476 * Math.pow(distance, 1) + 3300;
        // newDisRPM = 3.45238 * Math.pow(distance, 3) - 56 * Math.pow(distance, 2)
        // + 345 * Math.pow(distance, 1) + 2975;
        if (Math.abs(curDisRPM - newDisRPM) >= 100) {
            curDisRPM = newDisRPM;
            if (curDisRPM >= 6000) {
                curDisRPM = 6000 / 60;
            } else if (curDisRPM <= 3500) {
                curDisRPM = 3500 / 60;
            }
            this.shooter.setSetpoint(curDisRPM);
        }
    }
}
