package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * Climber subsytem, defines all climber motor and pneumatics
 */
public class Climber extends SubsystemBase {

    private final DoubleSolenoid outsideClimberSolenoid = new DoubleSolenoid(
        Constants.Pneumatics.pcm1, PneumaticsModuleType.CTREPCM,
        Constants.Pneumatics.Climber.forwardChannel1, Constants.Pneumatics.Climber.reverseChannel1);
    private final DoubleSolenoid insideClimberSolenoid = new DoubleSolenoid(
        Constants.Pneumatics.pcm2, PneumaticsModuleType.CTREPCM,
        Constants.Pneumatics.Climber.forwardChannel2, Constants.Pneumatics.Climber.reverseChannel2);

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

    public Climber() {
        outsideClimberMotor1.setInverted(true);
        insideClimberMotor1.setInverted(true);
    }

    public void initialize() {
        outsideClimberSolenoid.set(Value.kReverse);
        insideClimberSolenoid.set(Value.kReverse);
    }

    public void deployOutsideClimbers() {
        outsideClimberSolenoid.set(Value.kForward);
    }

    public void deployInsideClimbers() {
        insideClimberSolenoid.set(Value.kForward);
    }

    public void engageOutsideClimbers() {
        outsideMotors.set(.6);
    }

    public void disengageOutsideClimbers() {
        outsideMotors.set(0);
    }

    public void engageInsideClimbers() {
        insideMotors.set(.6);
    }

    public void disengageInsideClimbers() {
        insideMotors.set(0);
    }
}
