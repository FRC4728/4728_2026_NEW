package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.ShooterTable;
import frc.robot.subsystems.TurretShooter;


public class SetShooterByDistance extends Command {

    private final TurretShooter shooter;

    // Sanity clamp — ignore readings outside this range (inches)
    private static final double kMinDistance = 10.0;
    private static final double kMaxDistance = 300.0;

    
    private static final double kDeadband = 6.0;

    // Low-pass filter — smooths out noisy ty readings
    // 0.0 = no filtering, higher = more smoothing but slower to respond
    private static final double kFilterAlpha = 0.7;

    private double filteredDistance = -1;
    private double lastCommandedDistance = -1;
    private double lockedFlywheelRPS = 45.0;
    private double lockedHoodPosition = -3.0;

    public SetShooterByDistance(TurretShooter shooter) {
        this.shooter = shooter;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        filteredDistance = -1;
        lastCommandedDistance = -1;
        lockedFlywheelRPS = 45.0;
        lockedHoodPosition = -3.0;
    }

    @Override
    public void execute() {
        boolean hasTarget = LimelightHelpers.getTV("limelight-turret");

        if (!hasTarget) {
            // No target — hold last known setpoints, don't change anything
            shooter.runFlywheel(lockedFlywheelRPS);
            shooter.runHood(lockedHoodPosition);
            SmartDashboard.putNumber("Shooter/FilteredDistanceInches", filteredDistance);
            return;
        }

        double ty = LimelightHelpers.getTY("limelight-turret");
        double rawDistance = ShooterTable.getDistanceInches(ty);

        // Ignore bad readings
        if (rawDistance < kMinDistance || rawDistance > kMaxDistance) {
            shooter.runFlywheel(lockedFlywheelRPS);
            shooter.runHood(lockedHoodPosition);
            return;
        }

        // Initialize filter on first valid reading
        if (filteredDistance < 0) {
            filteredDistance = rawDistance;
        } else {
            filteredDistance = kFilterAlpha * filteredDistance + (1.0 - kFilterAlpha) * rawDistance;
        }

        // Only update setpoints if we've moved far enough to matter
        if (lastCommandedDistance < 0 || Math.abs(filteredDistance - lastCommandedDistance) > kDeadband) {
            lockedFlywheelRPS  = ShooterTable.getFlywheelRPS(filteredDistance);
            lockedHoodPosition = ShooterTable.getHoodPosition(filteredDistance);
            lastCommandedDistance = filteredDistance;

            SmartDashboard.putNumber("Shooter/TargetFlywheelRPS",  lockedFlywheelRPS);
            SmartDashboard.putNumber("Shooter/TargetHoodPosition", lockedHoodPosition);
        }

        shooter.runFlywheel(lockedFlywheelRPS);
        shooter.runHood(lockedHoodPosition);

        SmartDashboard.putNumber("Shooter/RawDistanceInches",      rawDistance);
        SmartDashboard.putNumber("Shooter/FilteredDistanceInches", filteredDistance);
        SmartDashboard.putNumber("Shooter/LastCommandedDistance",  lastCommandedDistance);
    }

    @Override
    public void end(boolean interrupted) {
        shooter.coastFlywheel();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}







