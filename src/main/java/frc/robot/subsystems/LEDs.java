package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * LED Subsystem
 */
public class LEDs extends SubsystemBase {
    private AddressableLED ledController;
    private AddressableLEDBuffer ledBuffer;
    public int pattern = 0;
    private int m_rainbowFirstPixelHue = 0;
    private int movingLED = 0;
    private boolean movingDirection = true;
    private int cylonEyeDelay = 0;
    private int policeDelay = 0;

    /**
     * Constructs an LED Subsystem object
     *
     * @param id PWM Port ID
     * @param length Number of addressable LEDs
     */
    public LEDs(int id, int length) {
        ledController = new AddressableLED(id);
        ledBuffer = new AddressableLEDBuffer(length);

        ledController.setLength(ledBuffer.getLength());
        ledController.setData(ledBuffer);
        ledController.start();
    }

    /**
     * Set the entire LED strip to a RGB color
     *
     * @param r Amount of red 0-255
     * @param g Amount of green 0-255
     * @param b Amount of blue 0-255
     */
    public void setColor(int r, int g, int b) {
        for (var i = 0; i < ledBuffer.getLength(); i++) {
            ledBuffer.setRGB(i, r, g, b);
        }
        ledController.setData(ledBuffer);
    }

    /**
     * Set the entire LED strip to a color
     *
     * @param color {@link Color} to set the strip
     */
    public void setColor(Color color) {
        for (var i = 0; i < ledBuffer.getLength(); i++) {
            ledBuffer.setLED(i, color);
        }
        ledController.setData(ledBuffer);
    }

    /**
     * Set the LED strip to a moving ranbow pattern
     */
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

    /**
     * Alternate the LED strip between Red and Blue like a police car
     */
    public void policeSirens() {
        if (policeDelay < 10) {
            for (var i = 0; i < 21; i++) {
                ledBuffer.setLED(i, Color.kRed);
            }
            for (var i = 21; i < ledBuffer.getLength(); i++) {
                ledBuffer.setLED(i, Color.kBlack);
            }
        } else {
            for (var i = 0; i < 21; i++) {
                ledBuffer.setLED(i, Color.kBlack);
            }
            for (var i = 21; i < ledBuffer.getLength(); i++) {
                ledBuffer.setLED(i, Color.kBlue);
            }
        }
        ledController.setData(ledBuffer);
        policeDelay += 1;
        policeDelay %= 21;
    }

    /**
     * Set the LED strip to a moving pixel back and forth of a single color
     *
     * @param color {@link Color} to set the pixel
     */
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

    /**
     * Set the LED strip to a moving pixel back and forth of a single color
     *
     * @param color {@link Color} to set the pixel
     * @param count number of LEDs to move
     * @param inverted Whether to invert the color choices
     */
    public void movingColor(Color color, int count, boolean inverted) {
        Color theColor = inverted ? Color.kBlack : color;
        Color background = inverted ? color : Color.kBlack;
        if (cylonEyeDelay == 0) {
            for (var i = 0; i < ledBuffer.getLength(); i++) {
                if (Math.abs(i - movingLED) < count) {
                    ledBuffer.setLED(i, theColor);
                } else {
                    ledBuffer.setLED(i, background);
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
        cylonEyeDelay += 1;
        cylonEyeDelay %= 2;
    }

    /**
     * Red moving pixel like a Cylon Eye
     */
    public void cylonEye() {
        if (cylonEyeDelay == 0) {
            this.movingColor(Color.kRed);
        }
        cylonEyeDelay += 1;
        cylonEyeDelay %= 3;
    }
}
