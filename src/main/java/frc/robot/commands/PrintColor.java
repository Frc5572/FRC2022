package frc.robot.commands;

import com.revrobotics.ColorSensorV3;
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
        ColorSensorV3.RawColor color = colorSensor.getRawColor();
        SmartDashboard.putNumber("Red", color.red);
        SmartDashboard.putNumber("Blue", color.blue);
        SmartDashboard.putNumber("Green", color.green);


        System.out.println(colorSensor.getColorString());
    }

}
