package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.modules.Vision;
import frc.robot.subsystems.Shooter;

/**
 * Controls Shooter RPM based on
 */
public class ShooterRPM extends CommandBase {

    private Shooter shooter;
    private Vision vision;

    public ShooterRPM(Shooter shooter, Vision vision) {
        this.shooter = shooter;
        this.vision = vision;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        this.shooter.enable();
    }

    @Override
    public void execute() {
        shooter.setSetpoint(this.vision.getShooterSpeed()); // IN RPS NOT RPM
    }

    @Override
    public void end(boolean interrupted) {
        this.shooter.disable();
    }
}
