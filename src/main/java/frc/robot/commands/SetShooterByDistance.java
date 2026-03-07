package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.ShooterTable;
import frc.robot.subsystems.TurretShooter;

/**
 * Continuously updates flywheel and hood setpoints based on distance to the AprilTag.
 * Uses ShooterTable to interpolate the correct values for the current distance.
 */
public class SetShooterByDistance extends Command {

    private final TurretShooter shooter;

    public SetShooterByDistance(TurretShooter shooter) {
        this.shooter = shooter;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        boolean hasTarget = LimelightHelpers.getTV("limelight-turret");

        if (hasTarget) {
            double ty = LimelightHelpers.getTY("limelight-turret");
            double distance = ShooterTable.getDistanceInches(ty);
            double flywheelRPS = ShooterTable.getFlywheelRPS(distance);
            double hoodPosition = ShooterTable.getHoodPosition(distance);

            shooter.runFlywheel(flywheelRPS);
            shooter.runHood(hoodPosition);

            SmartDashboard.putNumber("Shooter/DistanceInches", distance);
            SmartDashboard.putNumber("Shooter/TargetFlywheelRPS", flywheelRPS);
            SmartDashboard.putNumber("Shooter/TargetHoodPosition", hoodPosition);
        } else {
            // No target — stop flywheel and hold hood in place
            shooter.stopFlywheel();
            SmartDashboard.putNumber("Shooter/DistanceInches", -1);
        }
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stopFlywheel();
        shooter.stopHood();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}