package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Pneumatics;

/**
 * Defined intake motors and solenoids
 */
public class Intake extends SubsystemBase {
    WPI_TalonFX intakeMotor = new WPI_TalonFX(Constants.Motors.intakeMotorNum, "canivore");
    Solenoid intakeSol;
    private static final double intakeSpeed = .4;
    private static final int intakeStop = 0;

    /**
     * Constructs the Intake Subsystem
     *
     * @param ph PneumaticHub to create solenoids
     */
    public Intake(PneumaticHub ph) {
        this.intakeSol = ph.makeSolenoid(Pneumatics.intakeFowardChannel);
    }

    /**
     * Stops the intake from running and retracts the intake solenoid.
     */
    public void intakeRetract() {
        this.intakeMotor.set(intakeStop);
        // this.intakeSol.set(false);
    }

    /**
     * Runs the intake at a certain speed and extends the intake solenoid.
     */
    public void intakeDeploy() {
        this.intakeMotor.set(intakeSpeed);
        // this.intakeSol.set(true);
    }

    /**
     * Stops the intake from running.
     */
    public void stop() {
        this.intakeMotor.set(0);
    }
}
