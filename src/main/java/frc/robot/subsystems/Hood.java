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
    public Servo hoodServo = new Servo(Constants.HoodConstants.hoodServoID);
    Vision vision;
    double calculatedPosition;

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
        hoodServo.set(vision.getHoodValue());
        hoodServo.set(vision.getHoodValue());
    }

    public void setHoodPosition(double position) {
        hoodServo.set(position);
        hoodServo.set(position);
    }

    public String getCANCoderPos() {
        System.out.println(hoodCANCoder.getAbsolutePosition());
        return "CANCoder Pos: " + hoodCANCoder.getAbsolutePosition();
    }

    public void getServoPos() {
        // System.out.println(hoodServo.getPosition());
        System.out.println(hoodServo.getAngle());
        hoodServo.setAngle(270);
    }

}
