package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.MagazinePID;
import frc.robot.subsystems.Magazine;

/**
 * Creates commands for the magazine to run.
 */
public class MagControl extends PIDCommand {
    private Magazine magazine;

    /**
     * Initalizes the command for magazine and adds requirements to systems.
     */
    public MagControl(Magazine magazine) {
        super(new PIDController(MagazinePID.kP, MagazinePID.kI, MagazinePID.kD),
            () -> (magazine.getMeasurement()), () -> 10, output -> {
                magazine.periodic();
            });
        addRequirements(magazine);
    }

    public void initialize() {
        super.initialize();
    }

    @Override
    public boolean isFinished() {
        final Boolean interrupted = magazine.magSense.get();
        super.end(interrupted);
        return magazine.magSense.get();
    }
}
