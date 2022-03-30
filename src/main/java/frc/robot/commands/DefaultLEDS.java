package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LEDS;

/**
 * Creates LEDRainbow class.
 */
public class DefaultLEDS extends CommandBase {
    private final LEDS leds;
    private int redPulseBrightness = 0;
    private boolean direction = true;

    /**
     * Add requirements for LEDS.
     *
     * @param subsystem Adds to subsystem.
     */
    public DefaultLEDS(LEDS subsystem) {
        this.leds = subsystem;
        addRequirements(leds);
    }

    @Override
    public void execute() {
        if (leds.pattern == 0) {
            // Red pulse
            this.leds.setColor(redPulseBrightness, 0, 0);
            // increase brightness
            if (direction) {
                redPulseBrightness += 5;
            } else {
                redPulseBrightness -= 5;
            }
            if (redPulseBrightness >= 255 || redPulseBrightness <= 10) {
                direction = !direction;
            }
        } else if (leds.pattern == 1) {
            leds.rainbow();
        } else {
            if (leds.pattern == 2) {
                leds.policeSirens();
            }
        }



    }

}
