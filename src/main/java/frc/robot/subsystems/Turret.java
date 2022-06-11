package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.CANCoderStatusFrame;
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
    CANCoder turretCANCoder = new CANCoder(Constants.TurretConstants.turretCANCoderID, "canivore");
    CANCoderConfiguration turretCANCoderConfig = new CANCoderConfiguration();
    public static final double turretPower = Constants.TurretConstants.maxPower;
    public boolean alignEnabled = true;
    public boolean currentDirection = true; // True for CW, False for CCW
    public int rotations = 0;
    public int cancoderOffset = 20;
    public double previousCanCoderValue = getCANCoderPos();

    /**
     * Constructs the Turret Subsystem
     */
    public Turret() {
        turretMotor.setNeutralMode(NeutralMode.Coast);
        turretCANCoder.setStatusFramePeriod(CANCoderStatusFrame.SensorData, 1);
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
        previousCanCoderValue = newCanCoderValue;
        System.out.println(rotations);
    }

    /**
     * Moves turret to the left.
     */
    public void turretLeft() {
        turretMotor.set(-turretPower);
    }

    /**
     * Moves turret to the right.
     */
    public void turretRight() {
        turretMotor.set(turretPower);
    }

    /**
     * Set the turret to a certain power
     *
     * @param power Power to set the turret to
     */
    public void turretSet(double power) {
        turretMotor.set(power);
    }

    /**
     * Stops the turret from moving.
     */
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

