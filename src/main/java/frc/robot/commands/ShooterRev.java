package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

/**
 * Revs the shooter.
 */
public class ShooterRev extends CommandBase {
    private Shooter shooter;

    /**
     * Revs the shooter.
     *
     * @param shooter shooter subsystem
     */
    public ShooterRev(Shooter shooter) {
        this.shooter = shooter;
        addRequirements(shooter);
    }

    @Override
    public void execute() {
        this.shooter.spin();
    }



    @Override
    public void end(boolean interrupted) {
        this.shooter.stop();
    }
}
