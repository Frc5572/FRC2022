package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
<<<<<<< HEAD
=======
import edu.wpi.first.wpilibj.Servo;
>>>>>>> origin/main
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants;


/**
 * Creates subsystem and functions for the shooter.
 */
public class Shooter extends PIDSubsystem {
    private final WPI_TalonFX shooter = new WPI_TalonFX(Constants.Motors.shooterID);
<<<<<<< HEAD
    public static final double shooterSpin = 0.7;
=======
    private final Servo shooterServo = new Servo(Constants.Motors.shooterServoID);
    public static final double shooterSpin = 1.0;
>>>>>>> origin/main
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
        shooter.setInverted(true);
        shooter.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 1, 1);
<<<<<<< HEAD
        getController().setTolerance(50); // IN RPS NOT RPM
        setSetpoint(85); // IN RPS NOT RPM
=======
        getController().setTolerance(50);
        setSetpoint(85); // 5100 RPM, setpoint is in RPS not RPM
>>>>>>> origin/main
    }

    @Override
    public void useOutput(double output, double setpoint) {
        shooter.setVoltage(output + shooterFeed.calculate(setpoint));
    }

    @Override
    public double getMeasurement() {
        double selSenVel = shooter.getSelectedSensorVelocity(0);

<<<<<<< HEAD
        double RotPerSec = (double) selSenVel / Constants.ShooterPID.kUnitsPerRevolution
            * 10; /* scale per100ms to perSecond */
        return RotPerSec;
        // double RotPerMin = RotPerSec * 60.0;
        // return RotPerMin;
=======
        double rotPerSec = (double) selSenVel / Constants.ShooterPID.kUnitsPerRevolution
            * 10; /* scale per100ms to perSecond */

        // System.out.println("RPM (Speed): " + rotPerSec * 60);
        // System.out.println("Voltage: " + shooter.getMotorOutputVoltage());
        return rotPerSec;
>>>>>>> origin/main
    }

    @Override
    public void periodic() {
        if (m_enabled) {
            useOutput(m_controller.calculate(getMeasurement(), getSetpoint()), getSetpoint());
<<<<<<< HEAD
            double selSenVel = shooter.getSelectedSensorVelocity(0);
            double RotPerSec = (double) selSenVel / Constants.ShooterPID.kUnitsPerRevolution
                * 10; /* scale per100ms to perSecond */
            double RotPerMin = RotPerSec * 60.0;

            System.out.println("RPM (Speed): " + RotPerMin);
            System.out.println("Voltage: " + shooter.getMotorOutputVoltage());
=======
>>>>>>> origin/main
        }
    }

    public boolean atSetpoint() {
        return m_controller.atSetpoint();
<<<<<<< HEAD
    }

    public void spin() {
        shooter.set(shooterSpin);
    }

    public void stop() {
        shooter.set(motorStop);
=======
>>>>>>> origin/main
    }
}
