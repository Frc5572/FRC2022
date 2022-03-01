package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants;

/**
 * Creates and makes tasks for Magazine subsystem.
 */

public class Magazine extends PIDSubsystem {
    private WPI_TalonFX magazineMotor = new WPI_TalonFX(Constants.Motors.magazineMotorID);
    public DigitalInput magSense = new DigitalInput(1);
    private final SimpleMotorFeedforward magazineFeed = new SimpleMotorFeedforward(
        Constants.MagazinePID.kSVolts, Constants.MagazinePID.kVVoltSecondsPerRotation);

    /**
     * Initalize PID for the Magazine.
     */
    public Magazine() {
        super(new PIDController(Constants.MagazinePID.kP, Constants.MagazinePID.kI,
            Constants.MagazinePID.kD));
        magazineMotor.setInverted(true);
        magazineMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 1, 1);
        getController().setTolerance(Constants.MagazinePID.kMagazineToleranceRPS); // IN RPS NOT RPM
        setSetpoint(Constants.MagazinePID.kMagazineTargetRPS); // 3600 rpm - IN RPS NOT RPM
    }

    @Override
    public void useOutput(double output, double setpoint) {
        magazineMotor.setVoltage(output + magazineFeed.calculate(setpoint));
    }

    /**
     * Runs magazine down.
     */
    public void magazineDown() {
        magazineMotor.set(-.5);
    }

    /**
     * Stops magazine.
     */
    public void magazineStop() {
        magazineMotor.set(0);
    }

    @Override
    public double getMeasurement() {
        double selSenVel = magazineMotor.getSelectedSensorVelocity(0);

        double rotPerSec = (double) selSenVel / Constants.MagazinePID.kUnitsPerRevolution
            * 10; /* scale per100ms to perSecond */
        return rotPerSec;

        // System.out.println("RPM (Speed): " + rotPerSec * 60);
        // System.out.println("Voltage: " + magazineMotor.getMotorOutputVoltage());
    }

    @Override
    public void periodic() {
        if (m_enabled) {
            useOutput(m_controller.calculate(getMeasurement(), getSetpoint()), getSetpoint());

            // double selSenVel = magazineMotor.getSelectedSensorVelocity(0);

            // double rotPerSec = (double) selSenVel / Constants.MagazinePID.kUnitsPerRevolution
            // * 10; /* scale per100ms to perSecond */

            // System.out.println("RPM (Speed): " + rotPerSec * 60);
            // System.out.println("Voltage: " + magazineMotor.getMotorOutputVoltage());
            // System.out.println(magSense.get());
        }
    }

    public boolean atSetpoint() {
        return m_controller.atSetpoint();
    }
}
