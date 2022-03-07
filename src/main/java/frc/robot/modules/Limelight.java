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
    double h1 = Constants.VisionConstants.limelightHeight;
    double h2 = Constants.VisionConstants.targetHeight;
    double a1 = Constants.VisionConstants.limelightAngle;
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
        double a2 = state.yOffset;
        double distance = (h2 - h1) / java.lang.Math.tan(java.lang.Math.toRadians(a1 + a2));
        return distance;
    }

    /**
     * @return whether target is found
     */
    public boolean getTargetFound() {
        // Whether the limelight has any valid targets (0 or 1)
        return limelightTable.getEntry("tv").getDouble(0.0) == 1.0;
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
