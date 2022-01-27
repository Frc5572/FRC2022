package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.Vision;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class limelightAlign extends CommandBase{
    Swerve s_Swerve;
    Vision vision;
    Translation2d translation;

    public limelightAlign(Swerve s_Swerve, Vision vision){
        this.s_Swerve = s_Swerve;
        this.vision = vision;
        this.translation = new Translation2d(0, 0);
    }

    @Override
    public void initialize() {
        for(int i = 0; i < 50; i++){
            s_Swerve.drive(translation, .5, Constants.Swerve.isFieldRelative, Constants.Swerve.isOpenLoop);
        }
    }
    
    @Override
    public void execute() {
        s_Swerve.drive(translation, vision.getAimValue(), Constants.Swerve.isFieldRelative, Constants.Swerve.isOpenLoop);
    }

    @Override
    public boolean isFinished() {
        return vision.getTargetAligned();
    }

}
