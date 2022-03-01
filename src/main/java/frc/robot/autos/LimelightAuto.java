package frc.robot.autos;

import java.util.List;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.commands.AlignTurret;
import frc.robot.commands.ZeroMotorsWaitCommand;
import frc.robot.modules.AutoBase;
import frc.robot.modules.Vision;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Turret;

/**
 * Autonomous that aligns limelight then excecutes a trajectory.
 */
public class LimelightAuto extends AutoBase {
    Vision vision;
    Turret turret;

    /**
     * Autonomous that aligns limelight then excecutes a trajectory.
     *
     * @param swerve swerve subsystem
     * @param vision vision subsystem
     */
    public LimelightAuto(Swerve swerve, Turret turret, Vision vision) {
        super(swerve);
        this.turret = turret;
        System.out.println("Limelight Auto !!");
        TrajectoryConfig config =
            new TrajectoryConfig(Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                    .setKinematics(Constants.Swerve.swerveKinematics);

        Trajectory firstHalfTrajectory = TrajectoryGenerator.generateTrajectory(

            List.of(new Pose2d(0, 0, new Rotation2d(0)), new Pose2d(1, 0, new Rotation2d(0)),
                new Pose2d(1, 1, new Rotation2d(0))),
            config);

        SwerveControllerCommand firstHalfTraject = baseSwerveCommand(firstHalfTrajectory);
        ZeroMotorsWaitCommand firstWait = new ZeroMotorsWaitCommand(swerve, 3);
        ZeroMotorsWaitCommand secondWait = new ZeroMotorsWaitCommand(swerve, .5);
        AlignTurret align = new AlignTurret(turret, vision);

        addCommands(
            new InstantCommand(() -> swerve.resetOdometry(firstHalfTrajectory.getInitialPose())),
            align, firstHalfTraject, firstWait, secondWait);
    }
}
