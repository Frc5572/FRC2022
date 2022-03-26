package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

/**
 * This command will turn the robot to a specified angle.
 */
public class TurnToAngle extends ProfiledPIDCommand {

    private Swerve swerve;

    /**
     * Turns robot to specified angle.
     *
     * @param swerve Swerve subsystem
     * @param angle Requested angle to turn to
     */

    public TurnToAngle(Swerve swerve, double angle) {
        super(
            new ProfiledPIDController(Constants.Swerve.driveKP, Constants.Swerve.driveKI,
                Constants.Swerve.driveKD, new TrapezoidProfile.Constraints(360, 1080)),
            swerve::getRotation, angle, (output, setpoint) -> swerve.useOutput(output), swerve);
        getController().setTolerance(1);
        getController().enableContinuousInput(0, 360);
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        super.initialize();
    }

    @Override
    public void execute() {
        super.execute();
        swerve.drive(new Translation2d(0, 0), 0, Constants.Swerve.isOpenLoop,
            Constants.Swerve.isFieldRelative);
    }

    @Override
    public void end(boolean interrupt) {
        super.end(interrupt);
        swerve.useOutput(0);
        swerve.setMotorsZero(Constants.Swerve.isOpenLoop, Constants.Swerve.isFieldRelative);
    }

    @Override
    public boolean isFinished() {
        return getController().atGoal();
    }
}
