package frc.lib;

import static edu.wpi.first.wpilibj.util.ErrorMessages.requireNonNullParam;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class InfiniteCommand extends CommandBase {

    protected final Runnable m_onExecute;

    public InfiniteCommand(Runnable onExecute, Subsystem... requirements) {
        m_onExecute = requireNonNullParam(onExecute, "onExecute", "InfiniteCommand");

        addRequirements(requirements);
    }

    @Override
    public void execute() {
        m_onExecute.run();
    }
}
