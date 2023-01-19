package frc.robot.commands;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

/**
 * This command will turn the robot to a specified angle.
 */
public class TurnToAngle extends CommandBase {

    private Swerve swerve;
    private boolean isRelative;
    private double goal;
    private HolonomicDriveController holonomicDriveController;
    private Pose2d startPos = new Pose2d();
    private Pose2d targetPose2d = new Pose2d();

    /**
     * Turns robot to specified angle. Uses absolute rotation on field.
     *
     * @param swerve Swerve subsystem
     * @param angle Requested angle to turn to
     * @param isRelative Whether the angle is relative to the current angle: true = relative, false
     *        = absolute
     */

    public TurnToAngle(Swerve swerve, double angle, boolean isRelative) {
        addRequirements(swerve);
        this.swerve = swerve;
        this.goal = angle;
        this.isRelative = isRelative;

        PIDController xcontroller = new PIDController(Constants.AutoConstants.kPXController, 0, 0);
        PIDController ycontroller = new PIDController(Constants.AutoConstants.kPYController, 0, 0);
        ProfiledPIDController thetacontroller =
            new ProfiledPIDController(4, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
        holonomicDriveController =
            new HolonomicDriveController(xcontroller, ycontroller, thetacontroller);
        holonomicDriveController.setTolerance(new Pose2d(1, 1, Rotation2d.fromDegrees(1)));

    }

    @Override
    public void initialize() {
        startPos = swerve.getPose();
        if (isRelative) {
            targetPose2d = new Pose2d(startPos.getTranslation(),
                startPos.getRotation().rotateBy(Rotation2d.fromDegrees(goal)));
        } else {
            targetPose2d = new Pose2d(startPos.getTranslation(), Rotation2d.fromDegrees(goal));
        }
    }

    @Override
    public void execute() {
        Pose2d currPose2d = swerve.getPose();
        ChassisSpeeds chassisSpeeds = this.holonomicDriveController.calculate(currPose2d,
            targetPose2d, 0, targetPose2d.getRotation());
        SwerveModuleState[] swerveModuleStates =
            Constants.Swerve.swerveKinematics.toSwerveModuleStates(chassisSpeeds);
        swerve.setModuleStates(swerveModuleStates);
        // System.out.println(targetPose2d.relativeTo(currPose2d));
    }

    @Override
    public void end(boolean interrupt) {
        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0, 0, 0);
        SwerveModuleState[] swerveModuleStates =
            Constants.Swerve.swerveKinematics.toSwerveModuleStates(chassisSpeeds);
        swerve.setModuleStates(swerveModuleStates);
    }

    @Override
    public boolean isFinished() {
        return holonomicDriveController.atReference();

    }
}
