package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * Creates and makes tasks for Magazine subsystem.
 */

public class Magazine extends SubsystemBase {
    private WPI_TalonFX magazineMotor = new WPI_TalonFX(Constants.Motors.magazineMotorID);
    public DigitalInput magSense = new DigitalInput(2);

    public Magazine() {
        magazineMotor.setInverted(true);
    }

    public void startMagazine() {
        magazineMotor.set(.7);
        double selSenVel = magazineMotor.getSelectedSensorVelocity(0);
        double RotPerSec = (double) selSenVel / 2048 * 10; /* scale per100ms to perSecond */
        double RotPerMin = RotPerSec * 60.0;

        System.out.println("RPM (Speed): " + RotPerMin);
    }

    public void reverseMagazine() {
        magazineMotor.set(-.7);
    }

    public void stopMagazine() {
        magazineMotor.set(0);
    }
}
