package frc.robot.autos;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class TurnToAngle extends ProfiledPIDCommand {

    private Swerve swerve;

    public TurnToAngle(Swerve swerve, double angle) {
        super(
            new ProfiledPIDController(Constants.Swerve.angleKP, Constants.Swerve.angleKI,
                Constants.Swerve.angleKD, new TrapezoidProfile.Constraints(360, 1080)),
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
        swerve.drive(new Translation2d(0, 0), 0, true, true);
    }

    @Override
    public void end(boolean interrupt) {
        super.end(interrupt);
        swerve.useOutput(0);
        swerve.setMotorsZero(true, true);
    }

    @Override
    public boolean isFinished() {
        return getController().atGoal();
    }
}
