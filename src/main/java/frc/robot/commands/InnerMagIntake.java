package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.InnerMagazine;

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
    public void execute() {
        SmartDashboard.putBoolean("Magazine Switch", innerMagazine.magSense.get());
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
