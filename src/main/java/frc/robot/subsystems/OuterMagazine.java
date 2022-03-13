package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxRelativeEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants;

/**
 * Creates and makes tasks for Magazine subsystem.
 */

public class OuterMagazine extends PIDSubsystem {
    private CANSparkMax outerMagazineMotor =
        new CANSparkMax(Constants.Motors.outerMagazineMotorID, MotorType.kBrushless);
    private final SimpleMotorFeedforward magazineFeed = new SimpleMotorFeedforward(
        Constants.OuterMagazinePID.kSVolts, Constants.OuterMagazinePID.kVVoltSecondsPerRotation);
    private RelativeEncoder encoder =
        outerMagazineMotor.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);

    /**
     * Initalize PID for the Magazine.
     */
    public OuterMagazine() {
        super(new PIDController(Constants.OuterMagazinePID.kP, Constants.OuterMagazinePID.kI,
            Constants.OuterMagazinePID.kD));
        // getController().setTolerance(100);
        // setSetpoint(500);
        // encoder.setVelocityConversionFactor(.25);
        outerMagazineMotor.setInverted(false);
        outerMagazineMotor.setIdleMode(IdleMode.kBrake);
    }

    @Override
    public void useOutput(double output, double setpoint) {
        outerMagazineMotor.setVoltage(output + magazineFeed.calculate(setpoint));
    }

    /**
     * Runs magazine down.
     */
    public void magazineDown() {
        outerMagazineMotor.set(-.2);
    }

    /**
     * Runs magazine up.
     */
    public void magazineUp() {
        outerMagazineMotor.set(.2);
    }

    /**
     * Runs magazine up.
     */
    public void magazineUp(double power) {
        outerMagazineMotor.set(power);
    }

    /**
     * Stops magazine.
     */
    public void magazineStop() {
        outerMagazineMotor.set(0);
    }

    @Override
    public double getMeasurement() {
        double rotPerSec = encoder.getVelocity();
        // System.out.println("RPM (Speed): " + rotPerSec);
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
