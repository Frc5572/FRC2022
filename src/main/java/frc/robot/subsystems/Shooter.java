package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxRelativeEncoder;
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
    private final CANSparkMax shooterRoller =
        new CANSparkMax(Constants.Motors.shooterRollerID, MotorType.kBrushless);
    private RelativeEncoder encoder =
        shooterRoller.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);

    /**
     * Create shooter class for PID
     */
    public Shooter() {
        super(new PIDController(Constants.ShooterPID.kP, Constants.ShooterPID.kI,
            Constants.ShooterPID.kD));
        shooter.setInverted(true);
        shooter.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 1, 1);
        getController().setTolerance(Constants.ShooterPID.kShooterToleranceRPS); // IN RPS NOT RPM
        shooterRoller.setInverted(true);
        setSetpoint(0); // IN RPS NOT RPM
        encoder.setVelocityConversionFactor(.25);
    }

    /**
     * Spins shooter.
     */
    public void spinShooter() {
        shooter.set(.4);
    }

    /**
     * Stops the shooter.
     */
    public void stopShooter() {
        shooter.set(0);
    }

    @Override
    public void useOutput(double output, double setpoint) {
        shooter.setVoltage(output + shooterFeed.calculate(setpoint));
    }

    @Override
    public double getMeasurement() {
        double selSenVel = shooter.getSelectedSensorVelocity(0);
        double rotPerSec = (double) selSenVel / Constants.ShooterPID.kUnitsPerRevolution
            * 10; /* scale per100ms to perSecond */

        // System.out.println("SHOOTER RPM (Speed): " + rotPerSec * 60);
        // System.out.println("Voltage: " + shooter.getMotorOutputVoltage());
        return rotPerSec;
    }

    @Override
    public void periodic() {
        if (m_enabled) {
            useOutput(m_controller.calculate(getMeasurement(), getSetpoint()), getSetpoint());

            // double selSenVel = shooter.getSelectedSensorVelocity(0);
            // double rotPerSec = (double) selSenVel / Constants.ShooterPID.kUnitsPerRevolution
            // * 10; /* scale per100ms to perSecond */

            // System.out.println("SHOOTER RPM (Speed): " + rotPerSec * 60);
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
        this.shooterRoller.set(1);
    }


    /**
     * Disable the Shooter Subsystem
     */
    public void disableShooter() {
        this.disable();
        this.shooterRoller.set(0);
    }

    public double getRollerRPM() {
        return encoder.getVelocity();
    }
}
