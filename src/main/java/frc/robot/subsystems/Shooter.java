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
    public static final double shooterSpin = 1.0;
    public static final int motorStop = 0;
    public static final double turretPower = 0.1;

    public Shooter() {
        shooter.setInverted(true);
    }

    public void spin() {
        shooter.set(shooterSpin);
    }

    public void stop() {
        shooter.set(motorStop);
    }

    public void turretUp() {
        shooterServo.set(turretPower);
    }

    public void turretDown() {
        shooterServo.set(-turretPower);
    }
}
