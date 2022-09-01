package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.modules.ColorSensor;

public class PrintColor extends CommandBase {
    private ColorSensor colorSensor;

    public PrintColor(ColorSensor colorSensor) {
        this.colorSensor = colorSensor;
        addRequirements(colorSensor);
    }

    @Override
    public void execute() {
        SmartDashboard.putString("Color", colorSensor.getColorString());
    }

}
