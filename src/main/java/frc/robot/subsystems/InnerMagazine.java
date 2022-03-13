package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants;

/**
 * Creates tasks for inner Magazine rollers.
 */

public class InnerMagazine extends PIDSubsystem {
    private WPI_TalonFX innerMagazineMotor =
        new WPI_TalonFX(Constants.Motors.innerMagazineMotorID, "canivore");
    public DigitalInput magSense = new DigitalInput(1);
    private final SimpleMotorFeedforward magazineFeed = new SimpleMotorFeedforward(
        Constants.InnerMagazinePID.kSVolts, Constants.InnerMagazinePID.kVVoltSecondsPerRotation);

    /**
     * Initalize PID for the Magazine.
     */
    public InnerMagazine() {
        super(new PIDController(Constants.InnerMagazinePID.kP, Constants.InnerMagazinePID.kI,
            Constants.InnerMagazinePID.kD));
        innerMagazineMotor.setInverted(true);
        innerMagazineMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 1, 1);
        innerMagazineMotor.setNeutralMode(NeutralMode.Brake);
        getController().setTolerance(Constants.InnerMagazinePID.kInnerMagazineToleranceRPS); // IN
                                                                                             // RPS
                                                                                             // NOT
                                                                                             // RPM
        setSetpoint(Constants.InnerMagazinePID.kInnerMagazineTargetRPS); // 2000 rpm - IN RPS NOT
                                                                         // RPM
    }

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

    @Override
    public double getMeasurement() {
        double selSenVel = innerMagazineMotor.getSelectedSensorVelocity(0);

        double rotPerSec = (double) selSenVel / Constants.InnerMagazinePID.kUnitsPerRevolution
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
