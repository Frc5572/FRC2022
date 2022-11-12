package frc.robot.modules;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


/**
 * This is the public class ColorSensor.
 */
public class ColorSensor extends SubsystemBase {

    private final I2C.Port i2cPort = I2C.Port.kOnboard;
    private ColorSensorV3 colorSensor = new ColorSensorV3(i2cPort);
    public final ColorMatch colorMatcher = new ColorMatch();
    int i = 0;
    boolean that = false;

    private static Color rawColor(int r, int g, int b) {
        double mag = r + g + b;
        return new Color(r / mag, g / mag, b / mag);
    }

    private static final Color kRed = rawColor(8200, 4700, 1600);
    private static final Color kBlue = rawColor(1500, 4400, 5600);


    /**
     * This is ColorSensor.
     */
    public ColorSensor() {
        // Red Ball
        colorMatcher.addColorMatch(new Color(8200, 4700, 1600));
        // Yellow Shiny Ball
        colorMatcher.addColorMatch(new Color(18000, 31000, 4400));
        // Blue Ball
        colorMatcher.addColorMatch(new Color(1500, 4400, 5600));

        // colorMatcher.addColorMatch(Color.kGreen);
        // colorMatcher.addColorMatch(Color.kBlue);
        // colorMatcher.addColorMatch(Color.kRed);
        // colorMatcher.addColorMatch(Color.kYellow);
    }

    /**
     * This is GetColorString.
     */
    public Color colorMatch() {
        Color detectedColor = colorSensor.getColor();
        ColorMatchResult match = colorMatcher.matchClosestColor(detectedColor);
        // System.out.println(match.confidence);
        return match.color;
    }



    public String getColorString() {
        String colorString;

        Color color = colorMatch();

        if (color == Color.kBlue)

        {
            colorString = "Blue";
        } else if (color == Color.kRed) {
            colorString = "Red";
        } else if (color == Color.kGreen) {
            colorString = "Green";
        } else if (color == Color.kYellow) {
            colorString = "Yellow";
        } else if (color == Color.kBlue) {
            colorString = "Blue";
        } else {
            colorString = "Unknown";
        }
        return colorString;
    }

    public DriverStation.Alliance getBallColor() {
        Color color = colorMatch();

        if (color.equals(kRed)) {
            // SmartDashboard.putString("Red:", "Red");
            return DriverStation.Alliance.Red;
        } else if (color.equals(kBlue)) {
            // SmartDashboard.putString("Blue:", "Blue");
            return DriverStation.Alliance.Blue;
        }
        return DriverStation.Alliance.Invalid;
    }

    public boolean shouldSpit() {
        SmartDashboard.putString("Executing", "" + that);
        i++;
        that = !that;
        // return !getBallColor().equals(DriverStation.getAlliance());
        return !getBallColor().equals(DriverStation.getAlliance());

    }

    /**
     * This is ColorMatchResult, it returns the color matched by ColorMatch.
     */
    // public ColorMatchResult getColor() {

    // Color detectedColor = colorSensor.getColor();
    // ColorMatchResult match = colorMatcher.matchClosestColor(detectedColor);
    // return match;
    // }

    public ColorSensorV3.RawColor getRawColor() {
        return colorSensor.getRawColor();
    }
}
