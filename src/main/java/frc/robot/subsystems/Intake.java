package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
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
    private static final double intakeSpeed = .5;
    private static final int intakeStop = 0;

    /**
     * Constructs the Intake Subsystem
     *
     * @param ph PneumaticHub to create solenoids
     */
    public Intake(PneumaticHub ph) {
        super();
        this.intakeSol = ph.makeSolenoid(Pneumatics.intakeFowardChannel);
        intakeMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, 5000);
        intakeMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_Brushless_Current, 1000);
        intakeMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1, 5000);
        intakeMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 5000);
        intakeMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_14_Turn_PIDF1, 5000);
        intakeMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 1000);
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
