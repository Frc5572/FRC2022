package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;

/**
 * Creates a command for aligning the swerve drive during tele-op
 */
public class AlignSwerve extends CommandBase {

    private double rotation;
    private Translation2d translation;
    private boolean fieldRelative;
    private boolean openLoop;

    private Swerve swerveDrive;
    private Joystick controller;
    private int translationAxis;
    private int strafeAxis;
    private Vision vision;

    /**
     * Creates a command for aligning the swerve during teleop
     *
     * @param swerveDrive The instance of the swerve drive subsystem
     * @param controller The instance of the Driver Controller
     * @param translationAxis The forward-back axis
     * @param strafeAxis The left-right axis
     * @param fieldRelative Whether the movement is relative to the field or absolute
     * @param openLoop Open or closed loop system
     */
    public AlignSwerve(Swerve swerveDrive, Vision vision, Joystick controller, int translationAxis,
        int strafeAxis, boolean fieldRelative, boolean openLoop) {
        this.swerveDrive = swerveDrive;
        addRequirements(swerveDrive);
        this.vision = new Vision();
        this.controller = controller;
        this.translationAxis = translationAxis;
        this.strafeAxis = strafeAxis;
        this.fieldRelative = fieldRelative;
        this.openLoop = openLoop;
    }

    @Override
    public void execute() {
        vision.update();
        double yaxis = -controller.getRawAxis(translationAxis);
        double xaxis = -controller.getRawAxis(strafeAxis);

        /* Deadbands */
        yaxis = (Math.abs(yaxis) < Constants.stickDeadband) ? 0 : yaxis;
        xaxis = (Math.abs(xaxis) < Constants.stickDeadband) ? 0 : xaxis;

        translation = new Translation2d(yaxis, xaxis).times(Constants.Swerve.maxSpeed);
        rotation = vision.getAimValue();
        swerveDrive.drive(translation, rotation, fieldRelative, openLoop);
    }
}
