package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
    WPI_TalonSRX intakeMotor = new WPI_TalonSRX(IntakeConstants.intakeMotorNum);
    Solenoid intakeSol = new Solenoid(IntakeConstants.intakeModule, PneumaticsModuleType.CTREPCM, IntakeConstants.intakeFowardChannel);
}
