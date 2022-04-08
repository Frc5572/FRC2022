package frc.robot.commands;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LEDs;

/**
 * Creates LEDRainbow class.
 */
public class DefaultLEDs extends CommandBase {
    private final LEDs leds;
    private int redPulseBrightness = 0;
    private boolean direction = true;

    /**
     * Add requirements for LEDS.
     *
     * @param subsystem Adds to subsystem.
     */
    public DefaultLEDs(LEDs subsystem) {
        this.leds = subsystem;
        addRequirements(leds);
    }

    @Override
    public void execute() {

        if (leds.pattern == 0) {
            // Red pulse
            // this.leds.setColor(redPulseBrightness, 0, 0);
            // // increase brightness
            // if (direction) {
            // redPulseBrightness += 5;
            // } else {
            // redPulseBrightness -= 5;
            // }
            // if (redPulseBrightness >= 255 || redPulseBrightness <= 0) {
            // direction = !direction;
            // }
            leds.movingColor(Color.kGhostWhite, 5, false);
        } else if (leds.pattern == 1) {
            leds.rainbow();
        } else if (leds.pattern == 2) {
            leds.policeSirens();
        }
    }

}
