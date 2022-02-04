package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * Climber subsystem, defines all climber motor and pneumatics
 */
public class Climber extends SubsystemBase {

    private final Solenoid outsideClimberSolenoid = new Solenoid(Constants.Pneumatics.pcm1,
        PneumaticsModuleType.CTREPCM, Constants.Pneumatics.climberOutsideChannel);
    private final DoubleSolenoid insideClimberSolenoid =
        new DoubleSolenoid(Constants.Pneumatics.pcm2, PneumaticsModuleType.CTREPCM,
            Constants.Pneumatics.climberInsideChannel, Constants.Pneumatics.climberInsideChannel2);

    private final CANSparkMax outsideClimberMotor1 =
        new CANSparkMax(Constants.Motors.outsideClimberMotor1Id, MotorType.kBrushless);
    private final CANSparkMax outsideClimberMotor2 =
        new CANSparkMax(Constants.Motors.outsideClimberMotor2Id, MotorType.kBrushless);
    private final CANSparkMax insideClimberMotor1 =
        new CANSparkMax(Constants.Motors.insideClimberMotor1Id, MotorType.kBrushless);
    private final CANSparkMax insideClimberMotor2 =
        new CANSparkMax(Constants.Motors.insideClimberMotor2Id, MotorType.kBrushless);

    private final MotorControllerGroup outsideMotors =
        new MotorControllerGroup(outsideClimberMotor1, outsideClimberMotor2);
    private final MotorControllerGroup insideMotors =
        new MotorControllerGroup(insideClimberMotor1, insideClimberMotor2);

    private static final double motorSpeed = .6;
    private static final int motorStop = 0;

    // Sets motors to inverted control mode.
    public Climber() {
        outsideClimberMotor1.setInverted(true);
        insideClimberMotor1.setInverted(true);
    }

    // Initialize sets solenoids to false, or reverse, which makes it in the "Off" position
    // public void initialize() {
    // outsideClimberSolenoid.set(false);
    // insideClimberSolenoid.set(Value.kReverse);
    // }

    // This command will deploy the outside climbers solenoids.
    public void deployOutsideClimbers() {
        outsideClimberSolenoid.set(true);
    }

    // This command will deploy the inside climbers solenoids.
    public void deployInsideClimbers() {
        insideClimberSolenoid.set(Value.kForward);
    }

    // This command will start to move the outside climber's motors.
    public void engageOutsideMotors() {
        outsideMotors.set(motorSpeed);
    }

    // This command will stop moving the outside climber's motors.
    public void disengageOutsideMotors() {
        outsideMotors.set(-motorSpeed);
    }

    // This command will start to move the inside climber's motors.
    public void engageInsideMotors() {
        insideMotors.set(motorSpeed);
    }

    // This command will stop moving the inside climber's motors.
    public void disengageInsideMotors() {
        insideMotors.set(-motorSpeed);
    }

    // This will set the outside climber's pneumatics back to the default position.
    public void retractOutsideClimbers() {
        outsideClimberSolenoid.set(false);
    }

    // This will set the inside climber's pneumatics back to the default position .
    // (hopefully, not sure about double solenoids yet)
    public void retractInsideClimbers() {
        insideClimberSolenoid.set(Value.kReverse);
    }

    public void stopInsideMotors() {
        insideMotors.set(motorStop);
    }

    public void stopOutsideMotors() {
        outsideMotors.set(motorStop);
    }
}
