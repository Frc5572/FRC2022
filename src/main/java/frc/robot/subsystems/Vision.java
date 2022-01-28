package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * Vision subsystem.
 */

public class Vision extends SubsystemBase {

    double disX = 0;
    double disY = 0;
    double tx = 0;
    double ty = 0;
    double ta = 0;
    double tv = 0;
    boolean targetFound = false;
    double roundOff;

    double deadPocket = Constants.VisionConstants.deadPocket;

    /**
     *
     * @return align distance from center of target.
     *
     */

    public double update() {

        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
        tx = table.getEntry("tx").getDouble(0.0);
        ty = table.getEntry("ty").getDouble(0.0);
        ta = table.getEntry("ta").getDouble(0.0);
        tv = table.getEntry("tv").getDouble(0.0);

        if (tv < 1.0) {
            targetFound = false;
            disX = 0;
            disY = 0;
        } else {
            targetFound = true;
            disX = tx;
            disY = ty;
        }
        double calculated = -(disX / 125) * 10;
        roundOff = Math.round(calculated * 100.0) / 100.0;
        roundOff = (Math.abs(roundOff) <= deadPocket) ? 0 : roundOff;
        return roundOff;
    }

    public double getAimValue() {
        return update();
    }

    public boolean getTargetFound() {
        return targetFound;
    }

    public boolean getTargetAligned() {
        return roundOff == 0;
    }
}
