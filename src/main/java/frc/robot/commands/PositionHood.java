package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.modules.Vision;
import frc.robot.subsystems.Hood;

/**
 * PositionHood command will adjust hood based on preset angle or using Vision
 */

public class PositionHood extends ProfiledPIDCommand {
    private Hood hood;
    private Vision vision;
    private double oldAngle;
    private double requestedAngle;

    /**
     * PositionHood using requestedAngle. Only adjusts once, meaning the command will need to be
     * recalled if wanted again.
     *
     * @param hood Hood subsystem
     * @param requestedAngle The user's requested angle of the hood
     */
    public PositionHood(Hood hood, double requestedAngle) {
        super(new ProfiledPIDController(0, 0, 0, new TrapezoidProfile.Constraints(0, 0)),
            hood::getCANCoderPos, hood.calculateHoodPosition(requestedAngle),
            (output, setpoint) -> hood.useOutput(output));
        getController().setTolerance(1);
        addRequirements(hood);
        this.hood = hood;
    }

    /**
     * PositionHood using Vision module. Meant for a default command that is always running in order
     * to have to proper angle for shooting.
     * 
     * @param hood Hood subsystem
     * @param vision Vision subsystem
     */
    public PositionHood(Hood hood, Vision vision) {
        super(new ProfiledPIDController(0, 0, 0, new TrapezoidProfile.Constraints(0, 0)),
            hood::getCANCoderPos,
            hood.calculateHoodPosition(hood.calculateAngleFromDistance(vision.getDistance())),
            (output, setpoint) -> hood.useOutput(output));
        getController().setTolerance(1);
        addRequirements(hood);
        this.hood = hood;
        this.vision = vision;
    }

    @Override
    public void initialize() {
        if (this.vision == null) {
            getController().setGoal(this.requestedAngle);
        } else {
            getController().setGoal(
                hood.calculateHoodPosition(hood.calculateAngleFromDistance(vision.getDistance())));
        }
        super.initialize();
    }

    @Override
    public void execute() {
        if (this.vision != null) {
            getController().setGoal(
                hood.calculateHoodPosition(hood.calculateAngleFromDistance(vision.getDistance())));
        }
        super.execute();
    }

    @Override
    public void end(boolean interrupt) {
        super.end(interrupt);
        hood.useOutput(0);
    }
}
