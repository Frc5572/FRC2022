package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxRelativeEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants;

/**
 * Creates subsystem and functions for the shooter roller.
 */
public class ShooterRoller extends PIDSubsystem {
    private final CANSparkMax shooterRoller =
        new CANSparkMax(Constants.Motors.shooterRollerID, MotorType.kBrushless);
    private final SimpleMotorFeedforward shooterRollerFeed = new SimpleMotorFeedforward(
        Constants.ShooterRollerPID.kSVolts, Constants.ShooterRollerPID.kVVoltSecondsPerRotation);

    /**
     * Create shooter roller class for PID
     */
    public ShooterRoller() {
        super(new PIDController(Constants.ShooterRollerPID.kP, Constants.ShooterRollerPID.kI,
            Constants.ShooterRollerPID.kD));
        shooterRoller.setInverted(true); // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        // shooterRoller.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 1, 1);
        getController().setTolerance(Constants.ShooterRollerPID.kShooterRollerToleranceRPS); // IN
                                                                                             // RPS
                                                                                             // NOT
        // RPM
        setSetpoint(0); // IN RPS NOT RPM
    }

    /**
     * Spins shooter roller.
     */
    public void spinShooterRoller() {
        shooterRoller.set(.3);
    }

    /**
     * Stops the shooter roller.
     */
    public void stopShooterRoller() {
        shooterRoller.set(0);
    }

    @Override
    public void useOutput(double output, double setpoint) {
        shooterRoller.setVoltage(output + shooterRollerFeed.calculate(setpoint));
    }

    @Override
    public double getMeasurement() {
        RelativeEncoder encoder =
            shooterRoller.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);
        double selSenVel = encoder.getVelocity();
        // double selSenVel = shooterRoller.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor,
        // 42);
        double rotPerSec = (double) selSenVel / Constants.ShooterRollerPID.kUnitsPerRevolution
            * 10; /* scale per100ms to perSecond */

        // System.out.println("SHOOTER ROLLER RPM (Speed): " + rotPerSec * 60);
        // System.out.println("Voltage: " + shooterRoller.getMotorOutputVoltage());
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

    public boolean atSetpoint() {
        return m_controller.atSetpoint();
    }
}
