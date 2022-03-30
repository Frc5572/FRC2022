package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class LEDS extends SubsystemBase {
    private AddressableLED ledController;
    private AddressableLEDBuffer ledBuffer;
    public boolean pattern = false;

    public LEDS(int id) {
        ledController = new AddressableLED(id);
        ledBuffer = new AddressableLEDBuffer(21);

        ledController.setLength(ledBuffer.getLength());
        ledController.setData(ledBuffer);
        ledController.start();
    }

    public void setColor(int r, int g, int b) {
        for (var i = 0; i < ledBuffer.getLength(); i++) {
            ledBuffer.setRGB(i, r, g, b);
        }
        ledController.setData(ledBuffer);
    }


    public void setColor(Color color) {
        for (var i = 0; i < ledBuffer.getLength(); i++) {
            ledBuffer.setLED(i, color);
        }
        ledController.setData(ledBuffer);
    }

    public void rainbow() {
        int m_rainbowFirstPixelHue = 0;
        // For every pixel
        for (var i = 0; i < ledBuffer.getLength(); i++) {
            // Calculate the hue - hue is easier for rainbows because the color
            // shape is a circle so only one value needs to precess
            final var hue = (m_rainbowFirstPixelHue + (i * 180 / ledBuffer.getLength())) % 180;
            // Set the value
            ledBuffer.setHSV(i, hue, 255, 128);
        }
        // Increase by to make the rainbow "move"
        m_rainbowFirstPixelHue += 3;
        // Check bounds
        m_rainbowFirstPixelHue %= 180;
        ledController.setData(ledBuffer);
    }

    public void movingColor(Color color, int led) {
        for (var i = 0; i < ledBuffer.getLength(); i++) {
            if (i == led) {
                ledBuffer.setLED(i, color);
            } else {
                ledBuffer.setLED(i, Color.kBlack);
            }
        }
        ledController.setData(ledBuffer);
    }
}
