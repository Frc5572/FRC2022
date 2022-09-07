package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants;

/**
 * Creates tasks for inner Magazine rollers.
 */

public class InnerMagazine extends PIDSubsystem {
    public WPI_TalonFX innerMagazineMotor =
        new WPI_TalonFX(Constants.Motors.innerMagazineMotorID, "canivore");
    public DigitalInput magSense = new DigitalInput(Constants.InnerMagazinePID.sensorPWNPort);
    private final SimpleMotorFeedforward magazineFeed = new SimpleMotorFeedforward(
        Constants.InnerMagazinePID.kSVolts, Constants.InnerMagazinePID.kVVoltSecondsPerRotation);

    /**
     * Initialize PID for the Magazine.
     */
    public InnerMagazine() {
        super(new PIDController(Constants.InnerMagazinePID.kP, Constants.InnerMagazinePID.kI,
            Constants.InnerMagazinePID.kD));
        innerMagazineMotor.setInverted(true);
        innerMagazineMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 1, 1);
        innerMagazineMotor.setNeutralMode(NeutralMode.Brake);
        getController().setTolerance(Constants.InnerMagazinePID.kInnerMagazineToleranceRPS);
        setSetpoint(Constants.InnerMagazinePID.kInnerMagazineTargetRPS);
    }

    /**
     * Use feedforward voltage values to calculate the output.
     */
    @Override
    public void useOutput(double output, double setpoint) {
        innerMagazineMotor.setVoltage(output + magazineFeed.calculate(setpoint));
    }

    /**
     * Runs magazine down.
     */
    public void magazineDown() {
        innerMagazineMotor.set(-.2);
    }

    /**
     * Runs magazine up.
     */
    public void magazineUp() {
        innerMagazineMotor.set(.5);
    }

    /**
     * Stops magazine.
     */
    public void magazineStop() {
        innerMagazineMotor.set(0);
    }

    public DigitalInput getMagSense() {
        return magSense;
    }

    /**
     * Gets the rotations per second of the magazine.
     */
    @Override
    public double getMeasurement() {
        double selSenVel = innerMagazineMotor.getSelectedSensorVelocity(0);

        double rotPerSec = (double) selSenVel / Constants.InnerMagazinePID.kUnitsPerRevolution
            * 10; /* scale per100ms to perSecond */
        return rotPerSec;

        // System.out.println("RPM (Speed): " + rotPerSec * 60);
        // System.out.println("Voltage: " + magazineMotor.getMotorOutputVoltage());
    }

    /**
     * Sets the setpoint for the inner magazine using RPS and voltage calculations.
     */
    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Magazine Switch", magSense.get());
        if (m_enabled) {
            useOutput(m_controller.calculate(getMeasurement(), getSetpoint()), getSetpoint());
        }
    }

    public boolean atSetpoint() {
        return m_controller.atSetpoint();
    }
}
