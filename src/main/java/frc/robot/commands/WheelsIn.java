package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;

/**
 * Rotate all the wheels to the center
 */
public class WheelsIn extends CommandBase {
    Swerve swerve;

    /**
     * Rotate all the wheels to the center
     *
     * @param swerve Swerve Drive subsystem
     */
    public WheelsIn(Swerve swerve) {
        this.swerve = swerve;
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        this.swerve.wheelsIn();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
