package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * Class turret and all its functions.
 */
public class Turret extends SubsystemBase {
    private final WPI_TalonSRX turretMotor = new WPI_TalonSRX(Constants.Motors.turretMotorID);
    public static final double spinLeft = -0.2;
    public boolean alignEnabled = true;

    /**
     * Constructs the Turret Subsystem
     */
    public Turret() {
        turretMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, 5000);
        turretMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_Brushless_Current, 5000);
        turretMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1, 5000);
        turretMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 5000);
        turretMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_14_Turn_PIDF1, 5000);
        turretMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 10000);
        turretMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 10000);
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
}

