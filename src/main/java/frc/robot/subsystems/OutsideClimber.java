package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * Climber subsystem, defines all climber motor and pneumatics
 */
public class OutsideClimber extends SubsystemBase {

    private final Solenoid outsideClimberSolenoid;

    private final WPI_TalonFX outsideClimberMotor1 =
        new WPI_TalonFX(Constants.Motors.outsideClimberMotorRightId);
    private final WPI_TalonFX outsideClimberMotor2 =
        new WPI_TalonFX(Constants.Motors.outsideClimberMotorLeftId);
    private final WPI_TalonFX[] climberMotors =
        new WPI_TalonFX[] {outsideClimberMotor1, outsideClimberMotor2};

    private final MotorControllerGroup outsideMotors = new MotorControllerGroup(climberMotors);

    private static final double motorSpeed = .5;
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
            motor.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, 5000);
            motor.setStatusFramePeriod(StatusFrameEnhanced.Status_Brushless_Current, 1000);
            motor.setStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1, 5000);
            motor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 5000);
            motor.setStatusFramePeriod(StatusFrameEnhanced.Status_14_Turn_PIDF1, 5000);
            motor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 1000);
            motor.setNeutralMode(NeutralMode.Brake);
        }

        this.outsideClimberSolenoid = ph.makeSolenoid(Constants.Pneumatics.climberOutsideChannel);
        this.outsideClimberSolenoid.set(true);
    }

    /**
     * This command will deploy the outside climbers solenoids.
     */
    public void deployClimbers() {
        if (enabled) {
            this.outsideClimberSolenoid.set(true);
        }
    }

    /**
     * This command will start to move the outside climber's motors.
     */
    public void engageMotors() {
        if (enabled) {
            this.outsideMotors.set(motorSpeed);
        }
    }

    /**
     * This command will stop moving the outside climber's motors.
     */
    public void retractMotors() {
        if (enabled) {
            this.outsideMotors.set(-motorSpeed);
        }
    }

    /**
     * This will set the outside climber's pneumatics back to the default position.
     */
    public void retractClimbers() {
        if (enabled) {
            this.outsideClimberSolenoid.set(false);
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
