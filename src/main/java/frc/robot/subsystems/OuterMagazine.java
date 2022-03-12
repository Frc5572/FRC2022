package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants;

/**
 * Creates and makes tasks for Magazine subsystem.
 */

public class OuterMagazine extends PIDSubsystem {
    private WPI_TalonFX outerMagazineMotor =
        new WPI_TalonFX(Constants.Motors.outerMagazineMotorID, "canivore");
    private final SimpleMotorFeedforward magazineFeed = new SimpleMotorFeedforward(
        Constants.OuterMagazinePID.kSVolts, Constants.OuterMagazinePID.kVVoltSecondsPerRotation);

    /**
     * Initalize PID for the Magazine.
     */
    public OuterMagazine() {
        super(new PIDController(Constants.OuterMagazinePID.kP, Constants.OuterMagazinePID.kI,
            Constants.OuterMagazinePID.kD));
        outerMagazineMotor.setInverted(true);
        outerMagazineMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 1, 1);
        getController().setTolerance(Constants.OuterMagazinePID.kOuterMagazineToleranceRPS); // IN
                                                                                             // RPS
                                                                                             // NOT
                                                                                             // RPM
        setSetpoint(Constants.OuterMagazinePID.kOuterMagazineTargetRPS); // 2000 rpm - IN RPS NOT
                                                                         // RPM
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
        outerMagazineMotor.set(.5);
    }

    /**
     * Stops magazine.
     */
    public void magazineStop() {
        outerMagazineMotor.set(0);
    }

    @Override
    public double getMeasurement() {
        double selSenVel = outerMagazineMotor.getSelectedSensorVelocity(0);

        double rotPerSec = (double) selSenVel / Constants.OuterMagazinePID.kUnitsPerRevolution
            * 10; /* scale per100ms to perSecond */
        return rotPerSec;

        // System.out.println("RPM (Speed): " + rotPerSec * 60);
        // System.out.println("Voltage: " + magazineMotor.getMotorOutputVoltage());
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
