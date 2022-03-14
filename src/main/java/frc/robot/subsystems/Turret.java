package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * Class turret and all its functions.
 */
public class Turret extends SubsystemBase {
    private final WPI_TalonFX turretMotor =
        new WPI_TalonFX(Constants.Motors.turretMotorID, "canivore");
    public static final double spinLeft = -0.2;
    public boolean alignEnabled = true;

    public Turret() {
        turretBrakeMode(false);
    }

    public void turretLeft() {
        turretMotor.set(spinLeft);
    }

    public void turretRight() {
        turretMotor.set(-spinLeft);
    }

    public void turretSet(double power) {
        turretMotor.set(power);
    }

    public void turretStop() {
        turretMotor.set(0);
    }

    /**
     * Change the Idle mode of the motor
     *
     * @param mode True for brake mode, False for Coast
     */
    public void turretBrakeMode(boolean mode) {
        if (mode) {
            turretMotor.setNeutralMode(NeutralMode.Brake);
        } else {
            turretMotor.setNeutralMode(NeutralMode.Coast);
        }

    }
}

