package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.modules.Vision;

public class Hood extends SubsystemBase {
    Servo leftHoodServo = new Servo(0);
    Servo rightHoodServo = new Servo(0);
    Vision vision;

    public Hood(Vision vision) {
        this.vision = vision;
    }

    public void setHoodPosition() {
        leftHoodServo.set(vision.getDistance() / 100);
        rightHoodServo.set(vision.getDistance() / 100);
    }

    public void setHoodPosition(double position) {
        leftHoodServo.set(position);
        rightHoodServo.set(position);
    }

}
