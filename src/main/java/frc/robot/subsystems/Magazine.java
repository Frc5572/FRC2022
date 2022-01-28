package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Magazine extends SubsystemBase {
    private WPI_TalonSRX magazineMotor = new WPI_TalonSRX(Constants.magazinecanID);
    private boolean intaking;



    public void up() {
        intaking = true;
        magazineMotor.set(.5);

    }

    /*
     * public void stop() { magazineMotor.set(0); }
     */



    @Override
    protected boolean isFinished() {
        if (intaking) {
            return !Neptune.trident.infrared.get();
        } else {
            return isTimedOut();
            // ||Neptune.trident.infrared.get();
        }
    }

    @Override
    protected void end() {
        Neptune.trident.stopIntake();
    }



}
