package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.ShooterTable;
import frc.robot.subsystems.TurretShooter;

/**
 * Snapshots distance once when the command starts, then holds those
 * flywheel and hood setpoints for the duration of the shot.
 * This prevents noisy ty readings from causing the hood to flicker.
 */
public class SetShooterByDistance extends Command {

    private final TurretShooter shooter;

    // Sanity clamp — ignore readings outside this range (inches)
    private static final double kMinDistance = 10.0;
    private static final double kMaxDistance = 300.0;

    // Number of samples to average on startup for a stable distance reading
    private static final int kSampleCount = 10;

    private double lockedFlywheelRPS  = 45.0; // fallback defaults
    private double lockedHoodPosition = -3.0;
    private boolean hasLockedIn = false;

    public SetShooterByDistance(TurretShooter shooter) {
        this.shooter = shooter;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        hasLockedIn = false;
        lockedFlywheelRPS  = 45.0;
        lockedHoodPosition = -3.0;

        // Take several samples and average them for a stable initial distance
        if (LimelightHelpers.getTV("limelight-turret")) {
            double sum = 0;
            int validSamples = 0;

            for (int i = 0; i < kSampleCount; i++) {
                double ty = LimelightHelpers.getTY("limelight-turret");
                double dist = ShooterTable.getDistanceInches(ty);
                if (dist >= kMinDistance && dist <= kMaxDistance) {
                    sum += dist;
                    validSamples++;
                }
            }

            if (validSamples > 0) {
                double avgDistance = sum / validSamples;
                lockedFlywheelRPS  = ShooterTable.getFlywheelRPS(avgDistance);
                lockedHoodPosition = ShooterTable.getHoodPosition(avgDistance);
                hasLockedIn = true;

                SmartDashboard.putNumber("Shooter/LockedDistanceInches", avgDistance);
                SmartDashboard.putNumber("Shooter/TargetFlywheelRPS",    lockedFlywheelRPS);
                SmartDashboard.putNumber("Shooter/TargetHoodPosition",   lockedHoodPosition);
            }
        }
    }

    @Override
    public void execute() {
        // Just hold the locked setpoints — no continuous updates
        shooter.runFlywheel(lockedFlywheelRPS);
        shooter.runHood(lockedHoodPosition);

        SmartDashboard.putBoolean("Shooter/LockedIn", hasLockedIn);
    }

    @Override
    public void end(boolean interrupted) {
        shooter.coastFlywheel();
        shooter.stopHood();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}