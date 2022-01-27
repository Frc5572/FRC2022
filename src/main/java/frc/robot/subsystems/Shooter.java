package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * Creates subsystem and functions for the shooter.
 */
public class Shooter extends SubsystemBase {
    private final WPI_TalonSRX shooter = new WPI_TalonSRX(Constants.shooterID);
    private final Servo shooterServo = new Servo(Constants.shooterServoID);

    public void spin() {
        shooter.set(Constants.shooterSpin);
    }

    public void stop() {
        shooter.set(Constants.motorStop);
    }

    public void turretUp() {
        shooterServo.set(Constants.turretPower);
    }

    public void turretDown() {
        shooterServo.set(-Constants.turretPower);
    }
}
