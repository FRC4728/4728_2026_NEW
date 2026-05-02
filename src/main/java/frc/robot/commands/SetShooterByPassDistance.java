package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.PassTable;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.TurretShooter;

/**
 * Sets flywheel and hood based on distance to the pass target, with shoot-on-the-move
 * compensation identical to SetShooterByDistance.
 */
public class SetShooterByPassDistance extends Command {
    private final TurretShooter shooter;
    private final CommandSwerveDrivetrain drivetrain;
    private final Supplier<Translation2d> passTargetSupplier;

    private double filteredDistance = -1.0;
    private double lastCommandedDistance = -1.0;
    private double lockedFlywheelRPS = Constants.PassConstants.kPassFlywheelRPS;
    private double lockedHoodPosition = Constants.PassConstants.kPassHoodPosition;

    public SetShooterByPassDistance(
        TurretShooter shooter,
        CommandSwerveDrivetrain drivetrain,
        Supplier<Translation2d> passTargetSupplier
    ) {
        this.shooter = shooter;
        this.drivetrain = drivetrain;
        this.passTargetSupplier = passTargetSupplier;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        filteredDistance = -1.0;
        lastCommandedDistance = -1.0;
    }

    @Override
    public void execute() {
        Pose2d robotPose = drivetrain.getState().Pose;
        Translation2d target = passTargetSupplier.get();

        // Current distance used only to look up airtime for pose projection
        double rawDistanceMeters = robotPose.getTranslation().getDistance(target);
        double rawDistanceInches = Units.metersToInches(rawDistanceMeters);

        // Shoot-on-the-move: project robot to its future position
        double airtime = PassTable.getAirtime(rawDistanceInches);
        ChassisSpeeds fieldSpeeds = drivetrain.getFieldRelativeSpeeds();
        Translation2d futurePosition = new Translation2d(
            robotPose.getX() + fieldSpeeds.vxMetersPerSecond * airtime,
            robotPose.getY() + fieldSpeeds.vyMetersPerSecond * airtime
        );
        double futureDistanceMeters = futurePosition.getDistance(target);
        double futureDistanceInches = Units.metersToInches(futureDistanceMeters);

        // Sanity limits — fall back to locked values if out of range
        if (futureDistanceInches < Constants.PassConstants.kPassMinDistanceInches
            || futureDistanceInches > Constants.PassConstants.kPassMaxDistanceInches) {
            shooter.runFlywheel(lockedFlywheelRPS);
            shooter.runHood(lockedHoodPosition);
            return;
        }

        // Low-pass filter on distance to avoid rapid command changes
        if (filteredDistance < 0.0) {
            filteredDistance = futureDistanceInches;
        } else {
            filteredDistance =
                Constants.PoseAimConstants.kDistanceFilterAlpha * filteredDistance
                + (1.0 - Constants.PoseAimConstants.kDistanceFilterAlpha) * futureDistanceInches;
        }

        // Only re-command if distance moved past the deadband
        if (lastCommandedDistance < 0.0
            || Math.abs(filteredDistance - lastCommandedDistance)
                > Constants.PoseAimConstants.kShooterDistanceDeadbandInches) {

            lockedFlywheelRPS = PassTable.getFlywheelRPS(filteredDistance);
            lockedHoodPosition = PassTable.getHoodPosition(filteredDistance);
            lastCommandedDistance = filteredDistance;
        }

        shooter.runFlywheel(lockedFlywheelRPS);
        shooter.runHood(lockedHoodPosition);

        SmartDashboard.putNumber("Pass/RawDistanceInches", rawDistanceInches);
        SmartDashboard.putNumber("Pass/FutureDistanceInches", futureDistanceInches);
        SmartDashboard.putNumber("Pass/FilteredDistanceInches", filteredDistance);
        SmartDashboard.putNumber("Pass/TargetFlywheelRPS", lockedFlywheelRPS);
        SmartDashboard.putNumber("Pass/TargetHoodPosition", lockedHoodPosition);
        SmartDashboard.putNumber("Pass/Airtime", airtime);
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