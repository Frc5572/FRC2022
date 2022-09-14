package frc.robot.commands;

import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.modules.ColorSensor;

public class PrintBallColor extends CommandBase {
    private ColorSensor colorSensor;

    public PrintBallColor(ColorSensor colorSensor) {
        this.colorSensor = colorSensor;
        addRequirements(colorSensor);
    }

    @Override
    public void execute() {
        ColorSensorV3.RawColor color = colorSensor.getRawColor();
        SmartDashboard.putString("New Ball Color", colorSensor.getBallColor().toString());
        // SmartDashboard.putString("Ball Color", colorSensor.getBallColor().toString());
        // System.out.println(colorSensor.getBallColor().toString());
        SmartDashboard.putNumber("Green", color.green);
        SmartDashboard.putNumber("Blue", color.blue);
        SmartDashboard.putNumber("Red", color.red);


        // colorSensor.

        /*
         * Blue(rapid react ball):
         * 
         * 
         * 1in: R:670 G:1817 B:2193
         * 
         * .5in: R:970 G:2677 B:3355
         * 
         * touching: R:1572 G:4487 B:5615
         * 
         * 
         * Red(rapid react ball):
         * 
         * 
         * 1in: R:878 G:604 B:245
         * 
         * .5in: R:2229 G:508 B:1360
         * 
         * touching: R:8236 G:4746 B:1636
         * 
         * 
         * Yellow(shiny infinite recharge ball):
         * 
         * 
         * 1in: R:4558 G:7737 B:1231
         * 
         * .5in: R:13216 G:22408 B:3425
         * 
         * touching: R:18711 G:31612 B:4468
         */

        // System.out.println(colorSensor.getColorString());
    }

}
