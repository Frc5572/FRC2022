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
        updateSetpoint();
        this.shooter.enable();
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
        double distance = this.vision.getDistance() / 12;
        newDisRPM = (6 * Math.pow(distance, 2) + (90 * distance) + 3500); // IN RPS NOT RPM
        if (Math.abs(curDisRPM - newDisRPM) >= 50) {
            curDisRPM = newDisRPM;
            this.shooter.setSetpoint(curDisRPM > 6500 ? (6500 / 60) : (curDisRPM / 60));
        }
    }
}
