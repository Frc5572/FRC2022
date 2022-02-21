package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.modules.Vision;
import frc.robot.subsystems.Swerve;

/**
 * Command to allign swerve drivetrain with target using vision.
 */

public class LimelightAlign extends CommandBase {
    Swerve swerve;
    Vision vision;
    Translation2d translation;

    /**
     * Initializing variables.
     */
    public LimelightAlign(Swerve swerve, Vision vision) {
        this.swerve = swerve;
        this.vision = vision;
        this.translation = new Translation2d(0, 0);
    }

    @Override
    public void execute() {
        swerve.drive(translation, vision.getAimValue(), Constants.Swerve.isFieldRelative,
            Constants.Swerve.isOpenLoop);
    }

    @Override
    public boolean isFinished() {
        return vision.getTargetAligned();
    }

}
