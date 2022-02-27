package frc.lib;

import static edu.wpi.first.wpilibj.util.ErrorMessages.requireNonNullParam;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.Button;

public class AxisButton extends Button {
    private final GenericHID m_joystick;
    private final int m_axisNumber;
    private double m_sensitivity = 0.4;

    /**
     * Creates a joystick button for triggering commands.
     *
     * @param joystick The GenericHID object that has the button (e.g. Joystick, KinectStick, etc)
     * @param axisNumber The axis number (see {@link GenericHID#getRawAxis(int) }
     */
    public AxisButton(GenericHID joystick, int axisNumber) {
        requireNonNullParam(joystick, "joystick", "JoystickAxis");

        m_joystick = joystick;
        m_axisNumber = axisNumber;
    }

    /**
     * Creates a joystick button for triggering commands.
     *
     * @param joystick The GenericHID object that has the button (e.g. Joystick, KinectStick, etc)
     * @param axisNumber The axis number (see {@link GenericHID#getRawAxis(int) }
     * @param sensitivity The value the axis has to move to be considered as pressed
     */
    public AxisButton(GenericHID joystick, int axisNumber, double sensitivity) {
        requireNonNullParam(joystick, "joystick", "JoystickAxis");
        m_joystick = joystick;
        m_axisNumber = axisNumber;
        m_sensitivity = sensitivity;
    }

    /**
     * Gets the value of the joystick button.
     *
     * @return The value of the joystick button
     */
    @Override
    public boolean get() {
        return m_joystick.getRawAxis(m_axisNumber) > m_sensitivity;
    }
}
