package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TurretConstants;

/**
 * Class turret and all its functions.
 */
public class Turret extends SubsystemBase {
    private final WPI_TalonSRX turretMotor = new WPI_TalonSRX(TurretConstants.canId);

    public void turretLeft() {
        turretMotor.set(TurretConstants.spinLeft);
    }

    public void turretRight() {
        turretMotor.set(TurretConstants.spinRight);
    }

    public void turretStop() {
        turretMotor.set(TurretConstants.stop);
    }
}

