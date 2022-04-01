package frc.robot.subsystems;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
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
    double speed;
    double maxSpeed = 0.2;

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
        hoodMotor.setIdleMode(IdleMode.kBrake);
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
     * Set the Hood position
     */
    public void setHoodPosition() {
        speed = (Constants.HoodConstants.minPosition - getCANCoderPos()) / 300;
        if (speed > 0) {
            if (speed > maxSpeed) {
                hoodMotor.set(maxSpeed);
            } else {
                hoodMotor.set(speed);
            }
        } else {
            if (Math.abs(speed) > maxSpeed) {
                hoodMotor.set(-maxSpeed);
            } else {
                hoodMotor.set(speed);
            }
        }
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

    /**
     * Calculate required angle of the hood using Vision distance
     * 
     * @param distance The distance from the target reported by vision
     * @return The angle at which the hood needs to be set to
     */
    public double calculateAngleFromDistance(double distance) {
        double angle = 0;
        return angle;
    }

    public void setZero() {
        hoodMotor.set(0);
    }

    public void setOne() {
        hoodMotor.set(1);
    }

    public void setOneNeg() {
        hoodMotor.set(-1);
    }
}
