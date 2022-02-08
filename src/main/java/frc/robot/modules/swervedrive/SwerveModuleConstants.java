package frc.robot.modules.swerveDrive;

/**
 * Custom constants for each swerve module.
 */
public class SwerveModuleConstants {
    public final int driveMotorID;
    public final int angleMotorID;
    public final int cancoderID;
    public final double angleOffset;

    /**
     * @param driveMotorID CAN ID of the driving motor
     * @param angleMotorID CAN ID of the agnle motor
     * @param canCoderID CAN ID of the encoder
     * @param angleOffset Offset angle of something
     */

    public SwerveModuleConstants(int driveMotorID, int angleMotorID, int canCoderID,
        double angleOffset) {
        this.driveMotorID = driveMotorID;
        this.angleMotorID = angleMotorID;
        this.cancoderID = canCoderID;
        this.angleOffset = angleOffset;
    }
}
