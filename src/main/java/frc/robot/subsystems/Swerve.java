package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.SwerveModule;

/**
 * Creates swerve drive and commands for drive.
 */
public class Swerve extends SubsystemBase {
  public SwerveDriveOdometry swerveOdometry;
  public SwerveModule[] swerveMods;
  public AHRS gyro;

  /**
   * Initalizes swerve modules.
   */
  public Swerve() {
    gyro = new AHRS(Constants.Swerve.navXID);
    zeroGyro();

    swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getYaw());

    swerveMods = new SwerveModule[] {new SwerveModule(0, Constants.Swerve.Mod0.constants),
        new SwerveModule(1, Constants.Swerve.Mod1.constants),
        new SwerveModule(2, Constants.Swerve.Mod2.constants),
        new SwerveModule(3, Constants.Swerve.Mod3.constants)};
  }

  /**
   * Starts and creates drive subsystem.
   */
  public void drive(Translation2d translation, double rotation, boolean fieldRelative,
        boolean isOpenLoop) {
    SwerveModuleState[] swerveModuleStates =
            Constants.Swerve.swerveKinematics.toSwerveModuleStates(fieldRelative
                    ? ChassisSpeeds.fromFieldRelativeSpeeds(translation.getX(),
                            translation.getY(), rotation, getYaw())
                    : new ChassisSpeeds(translation.getX(), translation.getY(), rotation));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

  for (SwerveModule mod : swerveMods) {
        mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
    }
  }

  /**
   * Sets motors to 0 or inactive.
   */
  public void setMotorsZero(boolean isOpenLoop, boolean fieldRelative) {
    SwerveModuleState[] swerveModuleStates =
            Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                    fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, 0, getYaw())
                            : new ChassisSpeeds(0, 0, 0));

  for (SwerveModule mod : swerveMods) {
        mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
    }
    System.out.println("Setting Zero!!!!!!");
  }

/* Used by SwerveControllerCommand in Auto */
public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);

    for (SwerveModule mod : swerveMods) {
        mod.setDesiredState(desiredStates[mod.moduleNumber], false);
    }
}

public Pose2d getPose() {
    return swerveOdometry.getPoseMeters();
}

public void resetOdometry(Pose2d pose) {
    swerveOdometry.resetPosition(pose, getYaw());
}

public SwerveModuleState[] getStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (SwerveModule mod : swerveMods) {
        states[mod.moduleNumber] = mod.getState();
    }
    return states;
}

public void zeroGyro() {
    gyro.zeroYaw();
}

public Rotation2d getYaw() {
    float yaw = gyro.getYaw();
    return (Constants.Swerve.invertGyro) ? Rotation2d.fromDegrees(360 - yaw)
            : Rotation2d.fromDegrees(yaw);
}

@Override
public void periodic() {
    swerveOdometry.update(getYaw(), getStates());

    for (SwerveModule mod : swerveMods) {
        SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Cancoder",
                mod.getCanCoder().getDegrees());
        SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Integrated",
                mod.getState().angle.getDegrees());
        SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity",
                mod.getState().speedMetersPerSecond);
    }
  }
}
