package frc.robot.subsystems;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.modules.Vision;

/**
 * <<<<<<< HEAD
 *
 * Hood subsystem. =======
 * <p>
 * Hood subsystem.
 * </p>
 * >>>>>>> origin/main
 */

public class Hood extends SubsystemBase {
    CANCoderConfiguration hoodCanCoderConfig = new CANCoderConfiguration();
    CANCoder hoodCANCoder = new CANCoder(Constants.HoodConstants.hoodCANCoderID);
    public Servo hoodServo = new Servo(Constants.HoodConstants.hoodServoID);
    // public PWM test = new PWM(Constants.HoodConstants.hoodServoID);
    Vision vision;
    double calculatedPosition;

    /**
     * <<<<<<< HEAD
     *
     * Hood subsystem. =======
     * <p>
     * Hood subsystem.
     * </p>
     * >>>>>>> origin/main
     */

    public Hood(Vision vision) {
        this.vision = vision;
        /* hood CANCoder Configuration */
        hoodCanCoderConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        hoodCanCoderConfig.sensorDirection = Constants.HoodConstants.hoodCanCoderInvert;
        hoodCanCoderConfig.initializationStrategy =
            SensorInitializationStrategy.BootToAbsolutePosition;
        hoodCanCoderConfig.sensorTimeBase = SensorTimeBase.PerSecond;
        hoodServo.setBounds(2.5, 1.52, 1.5, 1.48, 0.5);
    }

    /**
     * <p>
     * Set hood position.
     * </p>
     */

    public void setHoodPosition(double position) {
        double error = position - hoodCANCoder.getAbsolutePosition();
        System.out.println(error);
        double speed = Math.abs(error) < 5 ? 0.0 : error < 0 ? .5 : -.5;
        System.out.println(speed);
        hoodServo.setSpeed(speed);
    }

    public double getCANCoderPos() {
        // System.out.println(hoodCANCoder.getAbsolutePosition());
        return hoodCANCoder.getAbsolutePosition();
    }

}
