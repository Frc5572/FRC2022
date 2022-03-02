package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.modules.Vision;
import frc.robot.subsystems.Shooter;

/**
 * Controls Shooter RPM based on
 */
public class ShooterRPM extends CommandBase {

    private Shooter shooter;
    private Vision vision;
    double curDisRPM = 0;
    double newDisRPM = 0;

    /**
     *
     * @param shooter shooter subsystem
     * @param vision vision subsystem
     */
    public ShooterRPM(Shooter shooter, Vision vision) {
        this.shooter = shooter;
        this.vision = vision;
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
        // System.out.println("SETPOINT: " + this.shooter.getSetpoint());
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
                + 402.6190476190476 * Math.pow(distance, 1) + 2800;
        // newDisRPM = (6 * Math.pow(distance, 2) + (90 * distance) + 3500); // IN RPS NOT RPM
        if (Math.abs(curDisRPM - newDisRPM) >= 100) {
            curDisRPM = newDisRPM;
            this.shooter.setSetpoint(curDisRPM > 6000 ? (6000 / 60) : (curDisRPM / 60));
        }
    }
}
