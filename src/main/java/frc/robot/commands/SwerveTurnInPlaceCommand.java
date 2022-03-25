package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

/**
 * Command for aligning turret in TeleOp
 */

public class SwerveTurnInPlaceCommand extends CommandBase {
    Swerve swerve;
    Rotation2d desiredPos;

    /**
     *
     * @param swerve swerve subsystem
     * @param desiredPos desired gyro position to turn to
     * 
     */
    public SwerveTurnInPlaceCommand(Swerve swerve, Rotation2d desiredPos) {
        this.swerve = swerve;
        this.desiredPos = desiredPos;
        addRequirements(swerve);
    }

    @Override
    public void execute() {
        // idk something like that
        if (Math.abs(swerve.getYaw().getDegrees()) - desiredPos.getDegrees() >= 2) {
            swerve.drive(new Translation2d(0, 0),
                Math.abs(swerve.getYaw().getDegrees()) - desiredPos.getDegrees(),
                Constants.Swerve.isFieldRelative, Constants.Swerve.isOpenLoop);
        }
    }

    @Override
    public void end(boolean interrupted) {
        swerve.drive(new Translation2d(0, 0), 0, Constants.Swerve.isFieldRelative,
            Constants.Swerve.isOpenLoop);
    }

    @Override
    public boolean isFinished() {
        return swerve.getYaw() == desiredPos;
    }
}
