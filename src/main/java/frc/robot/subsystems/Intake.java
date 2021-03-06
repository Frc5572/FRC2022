package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * Defined intake motors and solenoids
 */
public class Intake extends SubsystemBase {
    WPI_TalonFX intakeMotor = new WPI_TalonFX(Constants.Motors.intakeMotorNum, "canivore");
    private static final double intakeSpeed = .6;
    private static final int intakeStop = 0;

    /**
     * Constructs the Intake Subsystem
     */
    public Intake() {}

    /**
     * Stops the intake from running and retracts the intake solenoid.
     */
    public void intakeRetract() {
        this.intakeMotor.set(intakeStop);
    }

    /**
     * Runs the intake at a certain speed and extends the intake solenoid.
     */
    public void intakeDeploy() {
        this.intakeMotor.set(intakeSpeed);
    }

    /**
     * Runs the intake at a certain speed and extends the intake solenoid.
     */
    public void intakeDeploy(double speed) {
        this.intakeMotor.set(speed);
    }

    /**
     * Stops the intake from running.
     */
    public void stop() {
        this.intakeMotor.set(0);
    }
}
