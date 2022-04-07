package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.modules.Vision;
import frc.robot.subsystems.Shooter;

/**
 * Controls Shooter RPM based on
 */
public class ShooterRPM extends CommandBase {

    private Shooter shooter;
    private Vision vision;
    double curDisRPM = 0;
    double newDisRPM = 0;
    double setRPS;

    /**
     *
     * @param shooter shooter subsystem
     * @param vision vision subsystem
     */
    public ShooterRPM(Shooter shooter, Vision vision) {
        this.shooter = shooter;
        this.vision = vision;
        addRequirements(shooter);
    }

    /**
     *
     * @param shooter shooter subsystem
     * @param rps Hardcoded setpoint for Shooter
     */
    public ShooterRPM(Shooter shooter, double rps) {
        this.shooter = shooter;
        this.setRPS = rps;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        SmartDashboard.putNumber("SetPoint (RPM)", this.shooter.getSetpoint() * 60);
        // System.out.println("STARTING SHOOTER");
        // System.out.println("Initial RPM: " + this.shooter.getSetpoint());
        if (this.vision == null && this.setRPS > 0) {
            this.shooter.setSetpoint(this.setRPS);
        } else {
            updateSetpoint();
        }
        this.shooter.enableShooter();
    }

    @Override
    public void execute() {
        if (this.vision != null) {
            updateSetpoint();
        }
        // System.out.println("SHOOTER RPM: " + this.shooter.getRPM());
        // System.out.println("Roller RPM: " + this.shooter.getRollerRPM());

        // System.out.println("SHOOTER SETPOINT: " + this.shooter.getSetpoint());
    }

    @Override
    public void end(boolean interrupted) {
        curDisRPM = 0;
        shooter.setSetpoint(curDisRPM);
        this.shooter.disableShooter();
    }

    private void updateSetpoint() {
        double distance = this.vision.getDistance();
        // System.out.println("Vision Distance: " + distance);
        newDisRPM = -0.0237283391598 * Math.pow(distance, 2)
            + 20.7706947149615 * Math.pow(distance, 1) + 250;

        // newDisRPM =
        // 3.452380952381 * Math.pow(distance, 3) - 61.7857142857143 * Math.pow(distance, 2)
        // + 402.6190476190476 * Math.pow(distance, 1) + 3100;

        // 5.5ft - 3300rpm
        // 9ft - 4000rpm
        // 13.5ft - 4500rpm
        // 16.5ft - 5600rpm



        // newDisRPM = 3.45238 * Math.pow(distance, 3) - 51.42857 * Math.pow(distance, 2)
        // + 289.40476 * Math.pow(distance, 1) + 3300;
        // newDisRPM = 3.45238 * Math.pow(distance, 3) - 56 * Math.pow(distance, 2)
        // + 345 * Math.pow(distance, 1) + 2975;
        if (Math.abs(curDisRPM - newDisRPM) >= 100) {
            if (newDisRPM >= 4000) {
                curDisRPM = 4000;
            } else if (newDisRPM <= 2000) {
                curDisRPM = 2000;
            } else {
                curDisRPM = newDisRPM;
            }
            this.shooter.setSetpoint(curDisRPM / 60);
        }
    }
}
