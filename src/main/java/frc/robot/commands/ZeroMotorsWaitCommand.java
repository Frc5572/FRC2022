package frc.robot.commands;

import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

/**
 * Zeros motors then excecutes normal wait command.
 */
public class ZeroMotorsWaitCommand extends CommandBase {
    protected Timer timer = new Timer();
    private final double duration;
    private Swerve swerve;

    /**
     * Zeros motors then excecutes normal wait command.
     *
     * @param seconds how long the wait command should run
     */
    public ZeroMotorsWaitCommand(Swerve swerve, double seconds) {
        this.duration = seconds;
        SendableRegistry.setName(this, getName() + ": " + seconds + " seconds");
        this.swerve = swerve;
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        swerve.setMotorsZero(Constants.Swerve.isOpenLoop, Constants.Swerve.isFieldRelative);
        timer.reset();
        timer.start();
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(duration);
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }

}
