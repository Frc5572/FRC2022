package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
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
    private double yaxis;
    private double xaxis;
    private double raxis;
    private CommandXboxController controller;

    /**
     * Creates an command for driving the swerve drive during tele-op
     *
     * @param swerveDrive The instance of the swerve drive subsystem
     * @param fieldRelative Whether the movement is relative to the field or absolute
     * @param openLoop Open or closed loop system
     */
    public TeleopSwerve(Swerve swerveDrive, CommandXboxController controller, boolean fieldRelative,
        boolean openLoop) {
        this.swerveDrive = swerveDrive;
        addRequirements(swerveDrive);
        this.fieldRelative = fieldRelative;
        this.openLoop = openLoop;
        this.controller = controller;
    }

    @Override
    public void execute() {
        this.yaxis = -controller.getLeftY();
        this.xaxis = -controller.getLeftX();
        this.raxis = -controller.getRightX();

        /* Deadbands */
        yaxis = (Math.abs(yaxis) < Constants.stickDeadband) ? 0 : yaxis;
        xaxis = (Math.abs(xaxis) < Constants.stickDeadband) ? 0 : xaxis;
        raxis = (Math.abs(raxis) < Constants.stickDeadband) ? 0 : raxis;
        // System.out.println(swerveDrive.getStringYaw());

        translation = new Translation2d(yaxis, xaxis).times(Constants.Swerve.maxSpeed);
        rotation = raxis * Constants.Swerve.maxAngularVelocity;
        swerveDrive.drive(translation, rotation, fieldRelative, openLoop);
    }
}
