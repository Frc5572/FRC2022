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
import frc.robot.modules.Vision;

/**
 * <p>
 * Hood subsystem.
 * </p>
 */

public class Hood extends SubsystemBase {
    private final CANCoderConfiguration hoodCanCoderConfig = new CANCoderConfiguration();
    private final CANCoder hoodCANCoder =
        new CANCoder(Constants.HoodConstants.hoodCANCoderID, "canivore");
    private final CANSparkMax hoodMotor =
        new CANSparkMax(Constants.Motors.hoodMotorID, MotorType.kBrushless);
    Vision vision;
    double position;
    double calculatedPosition;

    /**
     * Hood subsystem.
     */

    public Hood(Vision vision) {
        this.vision = vision;
        /* hood CANCoder Configuration */
        hoodCanCoderConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        hoodCanCoderConfig.sensorDirection = Constants.HoodConstants.hoodCanCoderInvert;
        hoodCanCoderConfig.initializationStrategy =
            SensorInitializationStrategy.BootToAbsolutePosition;
        hoodCanCoderConfig.sensorTimeBase = SensorTimeBase.PerSecond;
    }

    /**
     * <p>
     * Set hood position.
     * </p>
     */

    public void setHoodPosition() {
        // replace this line with hood position calculation using
        // distance!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        position = vision.getDistance();
        calculatedPosition = Math.pow(position, 2) * position + 3000000;
        double error = position - hoodCANCoder.getAbsolutePosition();
        System.out.println(error);
        double speed = Math.abs(error) < 5 ? 0.0 : error < 0 ? .5 : -.5;
        System.out.println(speed);
    }

    public double getCANCoderPos() {
        // System.out.println(hoodCANCoder.getAbsolutePosition());
        return hoodCANCoder.getAbsolutePosition();
    }

}
