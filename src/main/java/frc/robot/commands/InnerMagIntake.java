package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.InnerMagazine;

/**
 * Command to run outer magazine for intake
 */
public class InnerMagIntake extends CommandBase {
    InnerMagazine innerMagazine;

    public InnerMagIntake(InnerMagazine innerMagazine) {
        this.innerMagazine = innerMagazine;
        addRequirements(innerMagazine);
    }

    @Override
    public void initialize() {
        this.innerMagazine.enable();
    }

    @Override
    public void end(boolean interrupted) {
        this.innerMagazine.disable();
    }

    @Override
    public boolean isFinished() {
        return innerMagazine.magSense.get();
    }
}
