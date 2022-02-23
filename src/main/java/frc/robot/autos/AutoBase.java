package frc.robot.autos;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

/**
 * Parent class for all autonomous commands
 */
public class AutoBase extends SequentialCommandGroup {
    Swerve swerve;
    ProfiledPIDController thetaController =
        new ProfiledPIDController(Constants.AutoConstants.kPThetaController, 0, 0,
            Constants.AutoConstants.kThetaControllerConstraints);

    /**
     * Autonomous that aligns limelight then excecutes a trajectory.
     *
     * @param swerve swerve subsystem
     */
    public AutoBase(Swerve swerve) {
        this.swerve = swerve;
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
    }
}
