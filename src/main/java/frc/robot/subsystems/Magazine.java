package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * Creates and makes tasks for Magazine subsystem.
 */

public class Magazine extends SubsystemBase {
    private WPI_TalonSRX magazineMotor = new WPI_TalonSRX(Constants.Motors.magazineMotorID);
    public DigitalInput magSense = new DigitalInput(2);

    public Magazine() {
        magazineMotor.setInverted(true);
    }

    /*
     * public void stop() { magazineMotor.set(0); }
     */
    public void up() {
        magazineMotor.set(.2);
    }

    public void startMagazine() {
        magazineMotor.set(.5);
    }

    public void stopMagazine() {
        magazineMotor.set(0);
    }
}
