package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.ShooterTable;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.TurretShooter;

public class SetShooterByDistance extends Command {
    private final TurretShooter shooter;
    private final CommandSwerveDrivetrain drivetrain;
    private final Turret turret;

    private double filteredDistance = -1.0;
    private double lastCommandedDistance = -1.0;
    private double lockedFlywheelRPS = 45.0;
    private double lockedHoodPosition = -3.0;

    public SetShooterByDistance(TurretShooter shooter, CommandSwerveDrivetrain drivetrain, Turret turret) {
        this.shooter = shooter;
        this.drivetrain = drivetrain;
        this.turret = turret;
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
        Translation2d target = turret.getAllianceTarget();

        double rawDistanceMeters = robotPose.getTranslation().getDistance(target);
        double rawDistanceInches = Units.metersToInches(rawDistanceMeters);

        if (rawDistanceInches < Constants.PoseAimConstants.kMinDistanceInches
            || rawDistanceInches > Constants.PoseAimConstants.kMaxDistanceInches) {
            shooter.runFlywheel(lockedFlywheelRPS);
            shooter.runHood(lockedHoodPosition);
            return;
        }

        if (filteredDistance < 0.0) {
            filteredDistance = rawDistanceInches;
        } else {
            filteredDistance =
                Constants.PoseAimConstants.kDistanceFilterAlpha * filteredDistance
                    + (1.0 - Constants.PoseAimConstants.kDistanceFilterAlpha) * rawDistanceInches;
        }

        if (lastCommandedDistance < 0.0
            || Math.abs(filteredDistance - lastCommandedDistance) > Constants.PoseAimConstants.kShooterDistanceDeadbandInches) {
            lockedFlywheelRPS = ShooterTable.getFlywheelRPS(filteredDistance);
            lockedHoodPosition = ShooterTable.getHoodPosition(filteredDistance);
            lastCommandedDistance = filteredDistance;
        }

        shooter.runFlywheel(lockedFlywheelRPS);
        shooter.runHood(lockedHoodPosition);

        SmartDashboard.putNumber("Shooter/RawDistanceInches", rawDistanceInches);
        SmartDashboard.putNumber("Shooter/FilteredDistanceInches", filteredDistance);
        SmartDashboard.putNumber("Shooter/TargetFlywheelRPS", lockedFlywheelRPS);
        SmartDashboard.putNumber("Shooter/TargetHoodPosition", lockedHoodPosition);
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
