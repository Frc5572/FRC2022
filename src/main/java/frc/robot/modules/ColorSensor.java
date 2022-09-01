package frc.robot.modules;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


/**
 * This is the public class ColorSensor.
 */
public class ColorSensor extends SubsystemBase {

    private final I2C.Port i2cPort = I2C.Port.kOnboard;
    private ColorSensorV3 colorSensor = new ColorSensorV3(i2cPort);
    private final ColorMatch colorMatcher = new ColorMatch();

    /**
     * This is ColorSensor.
     */
    public ColorSensor() {
        colorMatcher.addColorMatch(Color.kGreen);
        colorMatcher.addColorMatch(Color.kBlue);
        colorMatcher.addColorMatch(Color.kRed);
        colorMatcher.addColorMatch(Color.kYellow);
    }

    /**
     * This is GetColorString.
     */
    public Color colorMatch() {
        Color detectedColor = colorSensor.getColor();
        ColorMatchResult match = colorMatcher.matchClosestColor(detectedColor);
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
        } else {
            colorString = "Unknown";
        }
        return colorString;
    }

    public DriverStation.Alliance getBallColor() {
        Color color = colorMatch();

        if (color == Color.kBlue) {
            return DriverStation.Alliance.Blue;
        } else if (color == Color.kRed) {
            return DriverStation.Alliance.Red;
        }
        return DriverStation.Alliance.Invalid;
    }

    /**
     * This is ColorMatchResult, it returns the color matched by ColorMatch.
     */
    public ColorMatchResult getColor() {

        Color detectedColor = colorSensor.getColor();
        ColorMatchResult match = colorMatcher.matchClosestColor(detectedColor);
        return match;
    }
}
