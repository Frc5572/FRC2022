package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.modules.Vision;
import frc.robot.subsystems.Swerve;

/**
 * Creates an command for driving the swerve drive during tele-op
 */
public class TeleopSwerve extends CommandBase {

    private double rotation;
    private Translation2d translation;
    private boolean fieldRelative;
    private boolean openLoop;

    private Swerve swerveDrive;
    private Joystick controller;
    private int translationAxis;
    private int strafeAxis;
    private int rotationAxis;
    private Vision vision;

    /**
     * Creates an command for driving the swerve drive during tele-op
     *
     * @param swerveDrive The instance of the swerve drive subsystem
     * @param controller The instance of the Driver Controller
     * @param translationAxis The forward-back axis
     * @param strafeAxis The left-right axis
     * @param rotationAxis The rotation axis
     * @param fieldRelative Whether the movement is relative to the field or absolute
     * @param openLoop Open or closed loop system
     */
    public TeleopSwerve(Swerve swerveDrive, Vision vision, Joystick controller, int translationAxis,
        int strafeAxis, int rotationAxis, boolean fieldRelative, boolean openLoop) {
        this.swerveDrive = swerveDrive;
        addRequirements(swerveDrive);
        this.vision = new Vision();
        this.controller = controller;
        this.translationAxis = translationAxis;
        this.strafeAxis = strafeAxis;
        this.rotationAxis = rotationAxis;
        this.fieldRelative = fieldRelative;
        this.openLoop = openLoop;
    }

    @Override
    public void execute() {
        double yaxis = -controller.getRawAxis(translationAxis);
        double xaxis = -controller.getRawAxis(strafeAxis);
        double raxis = -controller.getRawAxis(rotationAxis);

        /* Deadbands */
        yaxis = (Math.abs(yaxis) < Constants.stickDeadband) ? 0 : yaxis;
        xaxis = (Math.abs(xaxis) < Constants.stickDeadband) ? 0 : xaxis;
        raxis = (Math.abs(raxis) < Constants.stickDeadband) ? 0 : raxis;

        translation = new Translation2d(yaxis, xaxis).times(Constants.Swerve.maxSpeed);
        rotation =
            (controller.getRawButton(XboxController.Button.kX.value) && vision.getTargetFound())
                ? vision.getAimValue()
                : raxis * Constants.Swerve.maxAngularVelocity;
        swerveDrive.drive(translation, rotation, fieldRelative, openLoop);
    }
}
