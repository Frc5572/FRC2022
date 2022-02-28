package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * Climber subsystem, defines all climber motor and pneumatics
 */
public class InsideClimber extends SubsystemBase {

    private final DoubleSolenoid insideClimberSolenoid;

    private final WPI_TalonFX insideClimberMotor1 =
        new WPI_TalonFX(Constants.Motors.insideClimberMotorRightId);
    private final WPI_TalonFX insideClimberMotor2 =
        new WPI_TalonFX(Constants.Motors.insideClimberMotorLeftId);
    private final WPI_TalonFX[] climberMotors =
        new WPI_TalonFX[] {insideClimberMotor1, insideClimberMotor2};

    private final MotorControllerGroup insideMotors = new MotorControllerGroup(climberMotors);

    private static final double motorSpeed = .5;
    private static final int motorStop = 0;
    private boolean enabled = false;

    /**
     * Constructs the Climber Subsystem
     *
     * @param ph PneumaticHub to create solenoids
     */
    public InsideClimber(PneumaticHub ph) {
        this.insideClimberMotor1.setInverted(true);
        for (WPI_TalonFX motor : climberMotors) {
            motor.setNeutralMode(NeutralMode.Brake);
        }

        this.insideClimberSolenoid = ph.makeDoubleSolenoid(
            Constants.Pneumatics.climberInsideChannel, Constants.Pneumatics.climberInsideChannel2);
        this.insideClimberSolenoid.set(Value.kReverse);
    }

    /**
     * This command will deploy the inside climbers solenoids.
     */
    public void deployClimbers() {
        if (enabled) {
            this.insideClimberSolenoid.set(Value.kReverse);
        }
    }

    /**
     * This command will start to move the inside climber's motors.
     */
    public void engageMotors() {
        if (enabled) {
            this.insideMotors.set(-motorSpeed);
        }
    }

    /**
     * This command will stop moving the inside climber's motors.
     */
    public void retractMotors() {
        if (enabled) {
            this.insideMotors.set(motorSpeed);
        }
    }

    /**
     * This will set the inside climber's pneumatics back to the default position. (hopefully, not
     * sure about double solenoids yet)
     */
    public void retractClimbers() {
        if (enabled) {
            this.insideClimberSolenoid.set(Value.kForward);
        }
    }

    /**
     * Stop the motors
     */
    public void stopMotors() {
        this.insideMotors.set(motorStop);
    }

    /**
     * Enable the use of the Climber
     */
    public void enableClimbers() {
        this.enabled = true;
    }
}
