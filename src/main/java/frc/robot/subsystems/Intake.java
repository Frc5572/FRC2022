package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Pneumatics;

/**
 * Defined intake motors and solenoids
 */
public class Intake extends SubsystemBase {
    WPI_TalonSRX intakeMotor = new WPI_TalonSRX(Constants.Motors.intakeMotorNum);
    Solenoid intakeSol;
    private static final double intakeSpeed = .7;
    private static final int intakeStop = 0;

    /**
     * Constructs the Intake Subsystem
     *
     * @param ph PneumaticHub to create solenoids
     */
    public Intake(PneumaticHub ph) {
        this.intakeSol = ph.makeSolenoid(Pneumatics.intakeFowardChannel);
    }

    public void intakeRetract() {
        this.intakeMotor.set(intakeStop);
        this.intakeSol.set(false);
    }

    public void intakeDeploy() {
        this.intakeMotor.set(intakeSpeed);
        this.intakeSol.set(true);
    }

    public void stop() {
        this.intakeMotor.set(0);
    }
}
