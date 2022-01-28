package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Magazine extends SubsystemBase {
    private WPI_TalonSRX magazineMotor = new WPI_TalonSRX(Constants.magazinecanID);



    public void startIntake() {
        magazineMotor.set(.5);

    }

    public void stopIntake() {
        magazineMotor.set(0);
    }



}
