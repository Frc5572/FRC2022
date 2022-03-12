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
    public static final double spinPower = -0.2;
    public boolean alignEnabled = true;

    public Turret() {
        turretMotor.setNeutralMode(NeutralMode.Brake);
    }

    public void turretLeft() {
        turretSet(spinPower);
    }

    public void turretRight() {
        turretSet(-spinPower);
    }

    public void turretSet(double power) {
        turretMotor.set(power);
    }

    public void turretStop() {
        turretMotor.set(0);
    }
}

