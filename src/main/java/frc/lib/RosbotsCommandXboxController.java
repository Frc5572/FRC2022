package frc.lib;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;

public class RosbotsCommandXboxController extends CommandXboxController{
    private final XboxController m_hid;

    /**
     * Construct an instance of a controller.
     *
     * @param port The port index on the Driver Station that the controller is plugged into.
     */
    public RosbotsCommandXboxController(int port) {
      super(port);
      m_hid = new XboxController(port);
    }

    public boolean getLeftTriggerPressed(){
        return Math.abs(getLeftTriggerAxis()) > Constants.triggerDeadband;
    }

    public boolean getRightTriggerPressed(){
        return Math.abs(getRightTriggerAxis()) > Constants.triggerDeadband;
    }

    /**
     * Constructs an event instance around the right trigger's digital signal.
     *
     * @return an event instance representing the right trigger's digital signal attached to the {@link
     *     CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
     * @see #rightTrigger(EventLoop)
     */
    public Trigger rightTrigger() {
      return new Trigger(() -> m_hid.getRightTriggerAxis() > Constants.triggerDeadband);
    }

    /**
     * Constructs an event instance around the left trigger's digital signal.
     *
     * @return an event instance representing the left trigger's digital signal attached to the {@link
     *     CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
     * @see #leftTrigger(EventLoop)
     */
    public Trigger leftTrigger() {
        return new Trigger(() -> m_hid.getLeftTriggerAxis() > Constants.triggerDeadband);
      }
}
