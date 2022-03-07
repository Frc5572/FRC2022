package frc.robot.modules;

import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;

/**
 * Vision Object.
 */

public class Limelight {
    NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
    double deadPocket = Constants.VisionConstants.deadPocket;
    double h1 = Constants.VisionConstants.limelightHeight;
    double h2 = Constants.VisionConstants.targetHeight;
    double a1 = Constants.VisionConstants.limelightAngle;
    int minAngle = Constants.HoodConstants.minAngle;
    int maxAngle = Constants.HoodConstants.maxAngle;
    double maxPosition = Constants.HoodConstants.maxPosition;
    double minPosition = Constants.HoodConstants.minPosition;
    double calculatedValue;
    double a2;
    double distance = 0.0;
    double disX = 0;
    double disY = 0;
    double tx = 0;
    double ty = 0;
    double tv = 0;
    double calculated;
    boolean targetFound = false;
    private volatile VisionState state = new VisionState(0, 0, 0);

    /**
     * Constructs a Vision object
     */
    public Limelight() {
        limelightTable.getEntry("tx").addListener(event -> {
            if (getTargetFound()) {
                double xOffset = limelightTable.getEntry("tx").getDouble(0);
                double yOffset = limelightTable.getEntry("ty").getDouble(0);
                double latency = limelightTable.getEntry("tl").getDouble(0) / 1000.0 + 0.011;
                state = new VisionState(xOffset, yOffset, Timer.getFPGATimestamp() - latency);
            }
        }, EntryListenerFlags.kUpdate);
    }

    /**
     * Get Vision State
     *
     * @return Vision state
     */
    public VisionState getState() {
        return state;
    }

    /**
     *
     * @return distance from target in inches
     *
     */
    public double getDistance() {
        // Vertical Offset From Crosshair To Target (LL1: -20.5 degrees to 20.5 degrees | LL2:
        // -24.85 to 24.85 degrees)
        a2 = state.yOffset;
        distance = (h2 - h1) / java.lang.Math.tan(java.lang.Math.toRadians(a1 + a2));
        return distance;
    }

    // /**
    // *
    // * @return distance from center of target
    // */

    // public double getAimValue() {
    // // Horizontal Offset From Crosshair To Target (LL1: -27 degrees to 27 degrees | LL2: -29.8
    // // to 29.8 degrees)
    // disX = state.xOffset;
    // double calculated = (disX / 125) * 3;
    // calculated =
    // (Math.abs(calculated) <= deadPocket) ? 0 : (calculated >= .3) ? .3 : calculated;
    // return calculated;
    // }

    /**
     * @return whether target is found
     */
    public boolean getTargetFound() {
        // Whether the limelight has any valid targets (0 or 1)
        return limelightTable.getEntry("tv").getDouble(0.0) == 1.0;
    }

    /**
     * @return whether target is aligned
     */
    public boolean getTargetAligned() {
        return calculated == 0;
    }

    /**
     * Turn the limielght LEDs off and On
     *
     * @param enabled
     */
    public void setLEDMode(boolean enabled) {
        int value = enabled ? 3 : 1;
        this.limelightTable.getEntry("ledMode").setNumber(value);
    }

    /**
     * State of the Limelight data
     */
    public static class VisionState {
        public final double xOffset;
        public final double yOffset;
        public final double timestamp;

        /**
         * State of the Limelight data
         */
        public VisionState(double xOffset, double yOffset, double timestamp) {
            this.xOffset = xOffset;
            this.yOffset = yOffset;
            this.timestamp = timestamp;
        }
    }
}
