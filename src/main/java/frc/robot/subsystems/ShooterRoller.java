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
        shooterRoller.setInverted(true);
        getController().setTolerance(Constants.ShooterRollerPID.kShooterRollerToleranceRPS);
        setSetpoint(0); // IN RPS NOT RPM
    }

    @Override
    public void useOutput(double output, double setpoint) {
        shooterRoller.setVoltage(output + shooterRollerFeed.calculate(setpoint));
    }

    @Override
    public double getMeasurement() {
        RelativeEncoder encoder =
            shooterRoller.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);
        encoder.setVelocityConversionFactor(0.5);
        double selSenVel = encoder.getVelocity();
        double rotPerSec = (double) selSenVel / 60;
        // System.out.println("SHOOTER ROLLER RPM (Speed): " + rotPerSec * 60);
        // System.out.println("Voltage: " + shooterRoller.getMotorOutputVoltage());
        return rotPerSec;
    }

    @Override
    public void periodic() {
        if (m_enabled) {
            useOutput(m_controller.calculate(getMeasurement(), getSetpoint()), getSetpoint());
        }
    }

    public boolean atSetpoint() {
        return m_controller.atSetpoint();
    }
}
