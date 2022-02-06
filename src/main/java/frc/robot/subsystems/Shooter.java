package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants;


/**
 * Creates subsystem and functions for the shooter.
 */
public class Shooter extends PIDSubsystem {
    private final WPI_TalonFX shooter = new WPI_TalonFX(Constants.Motors.shooterID);
    // private final Servo shooterServo = new Servo(Constants.Motors.shooterServoID);
    public static final double shooterSpin = 0.7;
    public static final int motorStop = 0;
    public static final double turretPower = 0.1;
    private final SimpleMotorFeedforward shooterFeed = new SimpleMotorFeedforward(
        Constants.ShooterPID.kSVolts, Constants.ShooterPID.kVVoltSecondsPerRotation);

    /**
     * Create shooter class for PID
     */
    public Shooter() {
        super(new PIDController(Constants.ShooterPID.kP, Constants.ShooterPID.kI,
            Constants.ShooterPID.kD));
        shooter.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 1, 1);
        getController().setTolerance(Constants.ShooterPID.kShooterToleranceRPS);
        setSetpoint(Constants.ShooterPID.kShooterTargetRPS);
    }

    @Override
    public void useOutput(double output, double setpoint) {
        shooter.setVoltage(output + shooterFeed.calculate(setpoint));
    }

    @Override
    public double getMeasurement() {
        return shooter.getSelectedSensorVelocity();
    }

    @Override
    public void periodic() {
        if (m_enabled) {
            useOutput(m_controller.calculate(getMeasurement(), getSetpoint()), getSetpoint());
            double selSenVel = shooter.getSelectedSensorVelocity(0);

            double RotPerSec = (double) selSenVel / Constants.ShooterPID.kUnitsPerRevolution
                * 10; /* scale per100ms to perSecond */
            double RotPerMin = RotPerSec * 60.0;

            System.out.println("RPM (Speed): " + RotPerMin);
            System.out.println("Voltage: " + shooter.getMotorOutputVoltage());
        }
    }

    public boolean atSetpoint() {
        return m_controller.atSetpoint();
    }

    public void spin() {
        shooter.set(shooterSpin);
    }

    public void stop() {
        shooter.set(motorStop);
    }

    // public void turretUp() {
    // shooterServo.set(turretPower);
    // }

    // public void turretDown() {
    // shooterServo.set(-turretPower);
    // }
}
