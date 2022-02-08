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
    private WPI_TalonFX magazine = new WPI_TalonFX(Constants.Motors.magazineMotorID);
    public DigitalInput magSense = new DigitalInput(2);
    private final SimpleMotorFeedforward magazineFeed = new SimpleMotorFeedforward(
        Constants.MagazinePID.kSVolts, Constants.MagazinePID.kVVoltSecondsPerRotation);

    public Magazine() {
        super(new PIDController(Constants.MagazinePID.kP, Constants.MagazinePID.kI,
            Constants.MagazinePID.kD));
        magazine.setInverted(true);
        magazine.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 1, 1);
        getController().setTolerance(50); // IN RPS NOT RPM
        setSetpoint(10); // IN RPS NOT RPM
    }

    @Override
    public void useOutput(double output, double setpoint) {
        magazine.setVoltage(output + magazineFeed.calculate(setpoint));
    }

    @Override
    public double getMeasurement() {
        double selSenVel = magazine.getSelectedSensorVelocity(0);

        double RotPerSec = (double) selSenVel / Constants.MagazinePID.kUnitsPerRevolution
            * 10; /* scale per100ms to perSecond */
        return RotPerSec;
    }

    @Override
    public void periodic() {
        if (m_enabled) {
            useOutput(m_controller.calculate(getMeasurement(), getSetpoint()), getSetpoint());
            double selSenVel = magazine.getSelectedSensorVelocity(0);
            double RotPerSec = (double) selSenVel / Constants.MagazinePID.kUnitsPerRevolution
                * 10; /* scale per100ms to perSecond */
            double RotPerMin = RotPerSec * 60.0;

            System.out.println("RPM (Speed): " + RotPerMin);
            System.out.println("Voltage: " + magazine.getMotorOutputVoltage());
        }
    }

    public boolean atSetpoint() {
        return m_controller.atSetpoint();
    }

    public void startMagazine() {
        magazine.set(.5);
    }

    public void stopMagazine() {
        magazine.set(0);
    }
}
