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
public class OutsideClimber extends SubsystemBase {

    private final DoubleSolenoid outsideClimberSolenoid;

    private final WPI_TalonFX outsideClimberMotor1 =
        new WPI_TalonFX(Constants.Motors.outsideClimberMotorRightId, "canivore");
    private final WPI_TalonFX outsideClimberMotor2 =
        new WPI_TalonFX(Constants.Motors.outsideClimberMotorLeftId, "canivore");
    private final WPI_TalonFX[] climberMotors =
        new WPI_TalonFX[] {outsideClimberMotor1, outsideClimberMotor2};

    private final MotorControllerGroup outsideMotors = new MotorControllerGroup(climberMotors);

    private static final double extendSpeed = .8;
    private static final double retractSpeed = -.5;
    private static final int motorStop = 0;
    private boolean enabled = false;

    /**
     * Constructs the Climber Subsystem
     *
     * @param ph PneumaticHub to create solenoids
     */
    public OutsideClimber(PneumaticHub ph) {
        this.outsideClimberMotor1.setInverted(true);
        for (WPI_TalonFX motor : climberMotors) {
            motor.setNeutralMode(NeutralMode.Brake);
        }
        this.outsideClimberSolenoid =
            ph.makeDoubleSolenoid(Constants.Pneumatics.climberOutsideForwardChannel,
                Constants.Pneumatics.climberOutsideReverseChannel);
        this.outsideClimberSolenoid.set(Value.kForward);
    }

    /**
     * This command will deploy the outside climbers solenoids.
     */
    public void deployClimbers() {
        if (enabled) {
            this.outsideClimberSolenoid.set(Value.kForward);
        }
    }

    /**
     * This command will start to move the outside climber's motors.
     */
    public void engageMotors() {
        if (enabled) {
            this.outsideMotors.set(extendSpeed);
        }
    }

    /**
     * This command will stop moving the outside climber's motors.
     */
    public void retractMotors() {
        if (enabled) {
            this.outsideMotors.set(retractSpeed);
        }
    }

    /**
     * This will set the outside climber's pneumatics back to the default position.
     */
    public void retractClimbers() {
        if (enabled) {
            this.outsideClimberSolenoid.set(Value.kReverse);
        }
    }

    /**
     * Stop the motors
     */
    public void stopMotors() {
        this.outsideMotors.set(motorStop);
    }

    /**
     * Enable the use of the Climber
     */
    public void enableClimbers() {
        this.enabled = true;
    }
}
