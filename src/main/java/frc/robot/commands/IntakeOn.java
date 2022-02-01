package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

/**
 * Initalizes the intake object for method use
 */
public class IntakeOn extends CommandBase {
    private final Intake intake;

    public IntakeOn(Intake subsytem) {
        this.intake = subsytem;
        addRequirements(intake);
    }

    @Override
    public void execute() {
        intake.intakeDeploy();
    }

    @Override
    public void end(boolean interrupted) {
        intake.intakeRetract();
    }

}