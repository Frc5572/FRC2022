package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.lib.math.Conversions;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

/**
 * This command will turn the robot to a specified angle.
 */
public class TurnToAngle extends ProfiledPIDCommand {

    private Swerve swerve;
    private boolean isRelative;
    private double goal;

    /**
     * Turns robot to specified angle. Uses absolute rotation on field.
     *
     * @param swerve Swerve subsystem
     * @param angle Requested angle to turn to
     * @param isRelative Whether the angle is relative to the current angle: true = relative, false
     *        = absolute
     * @param shortestPath Whether to enable continuous input to calculate the shortest path to turn
     */

    public TurnToAngle(Swerve swerve, double angle, boolean isRelative, boolean shortestPath) {
        super(
            // The ProfiledPIDController used by the command
            new ProfiledPIDController(
                // The PID gainss
                .01, 0, 0,
                // The motion profile constraints
                new TrapezoidProfile.Constraints(720, 1080)),
            // This should return the measurement
            swerve::getRotation,
            // This should return the goal (can also be a constant)
            isRelative ? Conversions.reduceTo0_360(swerve.getRotation() + angle)
                : Conversions.reduceTo0_360(angle),
            // This uses the output
            (output, setpoint) -> swerve.useOutput(output));
        // Use addRequirements() here to declare subsystem dependencies.
        // Configure additional PID options by calling `getController` here.
        getController().setTolerance(1);
        if (shortestPath) {
            getController().enableContinuousInput(0, 360);
        }
        addRequirements(swerve);
        this.swerve = swerve;
        this.isRelative = isRelative;
        this.goal = angle;
    }

    @Override
    public void initialize() {
        getController().setGoal(this.isRelative
            ? Conversions.reduceTo0_360(swerve.getRotation() + Conversions.reduceTo0_360(this.goal))
            : this.goal);
        super.initialize();
    }

    @Override
    public void execute() {
        super.execute();
        swerve.drive(new Translation2d(0, 0), 0, Constants.Swerve.isFieldRelative,
            Constants.Swerve.isOpenLoop);
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
