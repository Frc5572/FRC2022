package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class LEDs extends SubsystemBase {
    private AddressableLED ledController;
    private AddressableLEDBuffer ledBuffer;
    public int pattern = 0;
    private int m_rainbowFirstPixelHue = 0;
    private int movingLED = 0;
    private boolean movingDirection = true;
    private int cylonEyeDelay = 0;
    private int policeDelay = 0;

    public LEDs(int id) {
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

    public void policeSirens() {
        if (policeDelay < 5) {
            this.setColor(Color.kRed);
        } else {
            this.setColor(Color.kBlue);
        }
        policeDelay += 1;
        policeDelay %= 9;
    }

    public void movingColor(Color color) {
        for (var i = 0; i < ledBuffer.getLength(); i++) {
            if (i == movingLED) {
                ledBuffer.setLED(i, color);
            } else {
                ledBuffer.setLED(i, Color.kBlack);
            }
        }
        if (movingDirection) {
            movingLED += 1;
        } else {
            movingLED -= 1;
        }
        if (movingLED >= ledBuffer.getLength() - 1 || movingLED <= 0) {
            movingDirection = !movingDirection;
        }
        ledController.setData(ledBuffer);
    }

    public void cylonEye() {
        if (cylonEyeDelay == 0) {
            this.movingColor(Color.kRed);
        }
        cylonEyeDelay += 1;
        cylonEyeDelay %= 3;
    }
}
