package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * Class turret and all its functions.
 */
public class Turret extends SubsystemBase {
    private final WPI_TalonFX turretMotor =
        new WPI_TalonFX(Constants.Motors.turretMotorID, "canivore");
    CANCoder turretCANCoder = new CANCoder(Constants.HoodConstants.hoodCANCoderID, "canivore");
    CANCoderConfiguration turretCANCoderConfig = new CANCoderConfiguration();
    public static final double spinLeft = -0.2;
    public boolean alignEnabled = true;
    public boolean currentDirection = true; // True for CW, False for CCW
    public int rotations = 0;
    public int cancoderOffset = 20;
    public double previousCanCoderValue = getCANCoderPos();

    public Turret() {
        turretMotor.setNeutralMode(NeutralMode.Brake);
        turretCANCoderConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        turretCANCoderConfig.sensorDirection = Constants.HoodConstants.hoodCanCoderInvert;
        turretCANCoderConfig.initializationStrategy =
            SensorInitializationStrategy.BootToAbsolutePosition;
        turretCANCoderConfig.sensorTimeBase = SensorTimeBase.PerSecond;
    }

    @Override
    public void periodic() {
        double newCanCoderValue = getCANCoderPos();
        if (newCanCoderValue < 60 && previousCanCoderValue > 300) {
            rotations++;
        } else if (newCanCoderValue > 300 && previousCanCoderValue < 60) {
            rotations--;
        }
        if (newCanCoderValue - previousCanCoderValue > 0) {
            currentDirection = true;
        } else if (newCanCoderValue - previousCanCoderValue < 0) {
            currentDirection = false;
        }
        previousCanCoderValue = newCanCoderValue;
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

    public double getCANCoderPos() {
        // System.out.println(turretCANCoder.getAbsolutePosition());
        return turretCANCoder.getAbsolutePosition();
    }

    public double turretError() {
        return getCANCoderPos() - this.cancoderOffset;
    }
}

