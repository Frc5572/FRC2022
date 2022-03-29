package frc.robot.subsystems;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * Hood subsystem.
 */

public class Hood extends SubsystemBase {
    private final CANCoderConfiguration hoodCanCoderConfig = new CANCoderConfiguration();
    private final CANCoder hoodCANCoder =
        new CANCoder(Constants.HoodConstants.hoodCANCoderID, "canivore");
    private final CANSparkMax hoodMotor =
        new CANSparkMax(Constants.Motors.hoodMotorID, MotorType.kBrushless);
    double position;
    double calculatedPosition;
    double mOutput;

    /**
     * Hood subsystem.
     */

    public Hood() {
        /* hood CANCoder Configuration */
        hoodCanCoderConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        hoodCanCoderConfig.sensorDirection = Constants.HoodConstants.hoodCanCoderInvert;
        hoodCanCoderConfig.initializationStrategy =
            SensorInitializationStrategy.BootToAbsolutePosition;
        hoodCanCoderConfig.sensorTimeBase = SensorTimeBase.PerSecond;
    }

    /**
     * Calculate Hood Position using requested angle
     *
     * @param requestedAngle The requested angle at which the hood needs to be set to
     * @return The position of the CANCoder
     */
    public double calculateHoodPosition(double requestedAngle) {
        double rateOfChange =
            (Constants.HoodConstants.maxPosition - Constants.HoodConstants.minPosition)
                / (Constants.HoodConstants.maxAngle - Constants.HoodConstants.minAngle);
        double initOffset = Constants.HoodConstants.minPosition / rateOfChange * requestedAngle;
        return rateOfChange * requestedAngle + initOffset;
    }

    /**
     * Get CANCoder Position
     *
     * @return CANCoder position
     */
    public double getCANCoderPos() {
        return hoodCANCoder.getAbsolutePosition();
    }

    public void useOutput(double output) {
        hoodMotor.set(output);
    }

    public double calculateAngleFromDistance(double distance) {
        double angle = 0;
        return angle;
    }
}
