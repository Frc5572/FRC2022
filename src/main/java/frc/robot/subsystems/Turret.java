package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * Class turret and all its functions.
 */
public class Turret extends SubsystemBase {
    private final WPI_TalonSRX turretMotor = new WPI_TalonSRX(Constants.Motors.turretMotorID);
    public static final double spinLeft = -0.2;
    public boolean status = true;

    public void turretLeft() {
        turretMotor.set(spinLeft);
    }

    public void turretRight() {
        turretMotor.set(-spinLeft);
    }

    public void turretSet(double power) {
        turretMotor.set(power);
        System.out.println(status);
    }

    public void turretStop() {
        turretMotor.set(0);
    }
}

