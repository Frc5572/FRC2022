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
    private final WPI_TalonFX shooter = new WPI_TalonFX(Constants.Motors.shooterID, "canivore");
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
        getController().setTolerance(Constants.ShooterPID.kShooterToleranceRPS); // IN RPS NOT RPM
        setSetpoint(0); // IN RPS NOT RPM
    }

    /**
     * Spins shooter.
     */
    public void spinShooter() {
        shooter.set(.25);
    }

    /**
     * Stops the shooter.
     */
    public void stopShooter() {
        shooter.set(0);
    }

    /**
     * Use feedforward voltage values to calculate the output.
     */
    @Override
    public void useOutput(double output, double setpoint) {
        shooter.setVoltage(output + shooterFeed.calculate(setpoint));
    }

    /**
     * Get the current RPS of the shooter.
     */
    @Override
    public double getMeasurement() {
        double selSenVel = shooter.getSelectedSensorVelocity(0);
        double rotPerSec = (double) selSenVel / Constants.ShooterPID.kUnitsPerRevolution
            * 10; /* scale per100ms to perSecond */

        // System.out.println("SHOOTER RPM (Speed): " + rotPerSec * 60);
        // System.out.println("Voltage: " + shooter.getMotorOutputVoltage());
        return rotPerSec;
    }

    /**
     * Sets the setpoint for the magazine using RPS and voltage calculations.
     */
    @Override
    public void periodic() {
        if (m_enabled) {
            useOutput(m_controller.calculate(getMeasurement(), getSetpoint()), getSetpoint());
        }
    }

    /**
     * Get RPM of the Shooter motor
     *
     * @return RPM of the Shooter Motor
     */
    public double getRPM() {
        return getMeasurement() * 60;
    }

    /**
     * Checks if Shooter is within tolerance of the setpoint
     *
     * @return True if shooter is at the correct speed
     */
    public boolean atSetpoint() {
        return m_controller.atSetpoint();
    }

    /**
     * Enable the Shooter Subsystem
     */
    public void enableShooter() {
        this.enable();
    }


    /**
     * Disable the Shooter Subsystem
     */
    public void disableShooter() {
        this.disable();
    }
}
