package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * Climber subsytem, defines all climber motor and pneumatics
 */
public class Climber extends SubsystemBase {

    private final Solenoid outsideClimberSolenoid = new Solenoid(Constants.Pneumatics.pcm1,
        PneumaticsModuleType.CTREPCM, Constants.Pneumatics.Climber.outsideChannel);
    private final Solenoid insideClimberSolenoid = new Solenoid(Constants.Pneumatics.pcm2,
        PneumaticsModuleType.CTREPCM, Constants.Pneumatics.Climber.insideChannel);

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

    public Climber() {
        outsideClimberMotor1.setInverted(true);
        insideClimberMotor1.setInverted(true);
    }

    public void initialize() {
        outsideClimberSolenoid.set(false);
        insideClimberSolenoid.set(false);
    }

    public void deployOutsideClimbers() {
        outsideClimberSolenoid.set(true);
    }

    public void deployInsideClimbers() {
        insideClimberSolenoid.set(true);
    }

    public void engageOutsideClimbers() {
        outsideMotors.set(motorSpeed);
    }

    public void disengageOutsideClimbers() {
        outsideMotors.set(motorStop);
    }

    public void engageInsideClimbers() {
        insideMotors.set(motorSpeed);
    }

    public void disengageInsideClimbers() {
        insideMotors.set(motorStop);
    }
}
