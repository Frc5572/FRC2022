package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * Creates subsystem and functions for the shooter.
 */
public class Shooter extends SubsystemBase {
    private final WPI_TalonSRX shooter = new WPI_TalonSRX(Constants.Motors.shooterID);
    private final Servo shooterServo = new Servo(Constants.Motors.shooterServoID);

    public void spin() {
        shooter.set(Constants.Shooter.shooterSpin);
    }

    public void stop() {
        shooter.set(Constants.Shooter.motorStop);
    }

    public void turretUp() {
        shooterServo.set(Constants.Shooter.turretPower);
    }

    public void turretDown() {
        shooterServo.set(-Constants.Shooter.turretPower);
    }
}
