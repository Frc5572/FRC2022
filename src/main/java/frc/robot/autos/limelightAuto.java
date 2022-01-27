package frc.robot.autos;

import java.util.List;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.commands.ZeroMotorsWaitCommand;
import frc.robot.commands.limelightAlign;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;

public class limelightAuto extends SequentialCommandGroup {
    Vision vision;
    Swerve s_Swerve;

    public limelightAuto(Swerve s_Swerve, Vision vision) {
        this.s_Swerve = s_Swerve;
        System.out.println("Limelight Auto !!");
        TrajectoryConfig config =
            new TrajectoryConfig(Constants.AutoConstants.kMaxSpeedMetersPerSecond, Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared).setKinematics(Constants.Swerve.swerveKinematics);

        Trajectory firstHalfTrajectory = TrajectoryGenerator.generateTrajectory(

            List.of(new Pose2d(0, 0, new Rotation2d(0)), new Pose2d(1, 0, new Rotation2d(0)), new Pose2d(1, 1, new Rotation2d(0))), config);

        var thetaController = new ProfiledPIDController(Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        SwerveControllerCommand firstHalfTraject = new SwerveControllerCommand(firstHalfTrajectory, s_Swerve::getPose, Constants.Swerve.swerveKinematics,
            new PIDController(Constants.AutoConstants.kPXController, 0, 0), new PIDController(Constants.AutoConstants.kPYController, 0, 0), thetaController, s_Swerve::setModuleStates, s_Swerve);
        ZeroMotorsWaitCommand firstWait = new ZeroMotorsWaitCommand(3);
        ZeroMotorsWaitCommand secondWait = new ZeroMotorsWaitCommand(.5);
        limelightAlign align = new limelightAlign(s_Swerve, vision);

        addCommands(new InstantCommand(() -> s_Swerve.resetOdometry(firstHalfTrajectory.getInitialPose())), align, firstHalfTraject, firstWait, secondWait);
    }
}
