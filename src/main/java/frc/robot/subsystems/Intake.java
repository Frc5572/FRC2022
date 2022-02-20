package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Pneumatics;

/**
 * Defined intake motors and solenoids
 */
public class Intake extends SubsystemBase {
    WPI_TalonSRX intakeMotor = new WPI_TalonSRX(Constants.Motors.intakeMotorNum);
    Solenoid intakeSol =
        new Solenoid(Pneumatics.pcm1, PneumaticsModuleType.CTREPCM, Pneumatics.intakeFowardChannel);

    private static final double intakeSpeed = .5;
    private static final int intakeStop = 0;

    public void intakeRetract() {
        intakeMotor.set(intakeStop);
        intakeSol.set(false);
    }

    public void intakeDeploy() {
        intakeMotor.set(intakeSpeed);
        intakeSol.set(true);
    }

    public void in() {
        intakeMotor.set(.8);
    }

    public void stop() {
        intakeMotor.set(0);
    }
}
