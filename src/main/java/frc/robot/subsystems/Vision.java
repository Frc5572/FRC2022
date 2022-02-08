package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * Vision subsystem.
 */

public class Vision extends SubsystemBase {
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    double deadPocket = Constants.VisionConstants.deadPocket;
    double h1 = Constants.VisionConstants.limelightHeight;
    double h2 = Constants.VisionConstants.targetHeight;
    double a1 = Constants.VisionConstants.limelightAngle;
    double a2;
    double angle;
    double distance = 0.0;
    double disX = 0;
    double disY = 0;
    double tx = 0;
    double ty = 0;
    double ta = 0;
    double tv = 0;
    double calculated;
    boolean targetFound = false;

    /**
     *
     * @return distance from target in inches
     *
     */

    public double getDistance() {
        // Vertical Offset From Crosshair To Target (LL1: -20.5 degrees to 20.5 degrees | LL2:
        // -24.85 to 24.85 degrees)
        a2 = table.getEntry("ty").getDouble(0.0);
        distance = (h2 - h1) / java.lang.Math.tan(java.lang.Math.toRadians(a1 + a2));
        return distance;
    }

    /**
     *
     * @return distance from center of target
     */

    public double getAimValue() {
        // Horizontal Offset From Crosshair To Target (LL1: -27 degrees to 27 degrees | LL2: -29.8
        // to 29.8 degrees)
        tx = table.getEntry("tx").getDouble(0.0);
        targetFound = false;
        disX = 0;
        disX = tx;
        double calculated = -(disX / 125) * 10;
        calculated = (Math.abs(calculated) <= deadPocket) ? 0 : calculated;
        return calculated;
    }

    /**
     * @return whether target is found
     */

    public boolean getTargetFound() {
        // Whether the limelight has any valid targets (0 or 1)
        tv = table.getEntry("tv").getDouble(0.0);
        return tv == 1.0;
    }

    /**
     * @return whether target is aligned
     */
    public boolean getTargetAligned() {
        return calculated == 0;
    }
}