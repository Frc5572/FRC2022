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

public class Hood extends SubsystemBase {
    CANCoderConfiguration hoodCanCoderConfig = new CANCoderConfiguration();
    CANCoder hoodCANCoder = new CANCoder(Constants.HoodConstants.hoodCANCoderID);
    Servo hoodServo = new Servo(Constants.HoodConstants.hoodServoID);
    Vision vision;

    public Hood(Vision vision) {
        this.vision = vision;
        /* hood CANCoder Configuration */
        hoodCanCoderConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        hoodCanCoderConfig.sensorDirection = Constants.HoodConstants.hoodCanCoderInvert;
        hoodCanCoderConfig.initializationStrategy =
            SensorInitializationStrategy.BootToAbsolutePosition;
        hoodCanCoderConfig.sensorTimeBase = SensorTimeBase.PerSecond;
    }

    public void setHoodPosition() {
        hoodServo.set(vision.getDistance() / 100);
        hoodServo.set(vision.getDistance() / 100);
    }

    public void setHoodPosition(double position) {
        hoodServo.set(position);
        hoodServo.set(position);
    }

    public void getCANCoderPos() {
        System.out.println(hoodCANCoder.getAbsolutePosition());
    }

}
