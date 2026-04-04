package frc.robot.commands;
 
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
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
 
    public SetShooterByDistance(
        TurretShooter shooter,
        CommandSwerveDrivetrain drivetrain,
        Turret turret
    ) {
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
 
        // Current distance used only to look up airtime for pose projection
        double rawDistanceMeters = robotPose.getTranslation().getDistance(target);
        double rawDistanceInches = Units.metersToInches(rawDistanceMeters);
 
        // --- Shoot-on-the-move: project robot to its future position ---
        double airtime = ShooterTable.getAirtime(rawDistanceInches);
        ChassisSpeeds fieldSpeeds = drivetrain.getFieldRelativeSpeeds();
        Translation2d futurePosition = new Translation2d(
            robotPose.getX() + fieldSpeeds.vxMetersPerSecond * airtime,
            robotPose.getY() + fieldSpeeds.vyMetersPerSecond * airtime
        );
        double futureDistanceMeters = futurePosition.getDistance(target);
        double futureDistanceInches = Units.metersToInches(futureDistanceMeters);
        // ---------------------------------------------------------------
 
        if (futureDistanceInches < Constants.PoseAimConstants.kMinDistanceInches
            || futureDistanceInches > Constants.PoseAimConstants.kMaxDistanceInches) {
            shooter.runFlywheel(lockedFlywheelRPS);
            shooter.runHood(lockedHoodPosition);
            return;
        }
 
        if (filteredDistance < 0.0) {
            filteredDistance = futureDistanceInches;
        } else {
            filteredDistance =
                Constants.PoseAimConstants.kDistanceFilterAlpha * filteredDistance
                + (1.0 - Constants.PoseAimConstants.kDistanceFilterAlpha) * futureDistanceInches;
        }
 
        if (lastCommandedDistance < 0.0
            || Math.abs(filteredDistance - lastCommandedDistance)
                > Constants.PoseAimConstants.kShooterDistanceDeadbandInches) {
 
            lockedFlywheelRPS = ShooterTable.getFlywheelRPS(filteredDistance);
            lockedHoodPosition = ShooterTable.getHoodPosition(filteredDistance);
            lastCommandedDistance = filteredDistance;
        }
 
        shooter.runFlywheel(lockedFlywheelRPS);
        shooter.runHood(lockedHoodPosition);
 
        SmartDashboard.putNumber("Shooter/RawDistanceInches", rawDistanceInches);
        SmartDashboard.putNumber("Shooter/FutureDistanceInches", futureDistanceInches);
        SmartDashboard.putNumber("Shooter/FilteredDistanceInches", filteredDistance);
        SmartDashboard.putNumber("Shooter/LastCommandedDistance", lastCommandedDistance);
        SmartDashboard.putNumber("Shooter/TargetFlywheelRPS", lockedFlywheelRPS);
        SmartDashboard.putNumber("Shooter/TargetHoodPosition", lockedHoodPosition);
        SmartDashboard.putNumber("Shooter/Airtime", airtime);
        SmartDashboard.putNumber("Drivetrain/FieldVX", fieldSpeeds.vxMetersPerSecond);
        SmartDashboard.putNumber("Drivetrain/FieldVY", fieldSpeeds.vyMetersPerSecond);
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