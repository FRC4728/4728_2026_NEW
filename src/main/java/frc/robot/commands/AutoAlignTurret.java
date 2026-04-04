package frc.robot.commands;
 
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.ShooterTable;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Turret;
 
public class AutoAlignTurret extends Command {
    private final Turret m_turret;
    private final CommandSwerveDrivetrain m_drivetrain;
    private final Translation2d m_overrideTarget; // null = use alliance scoring target
 
    /** Standard constructor — aims at the alliance scoring target. */
    public AutoAlignTurret(Turret turret, CommandSwerveDrivetrain drivetrain) {
        this(turret, drivetrain, null);
    }
 
    /** Pass constructor — aims at a specific field position instead. */
    public AutoAlignTurret(Turret turret, CommandSwerveDrivetrain drivetrain, Translation2d target) {
        this.m_turret = turret;
        this.m_drivetrain = drivetrain;
        this.m_overrideTarget = target;
        addRequirements(m_turret);
    }
 
    @Override
    public void initialize() {
        SmartDashboard.putBoolean("Turret/CommandRunning", true);
    }
 
    @Override
    public void execute() {
        Pose2d robotPose = m_drivetrain.getState().Pose;
 
        Translation2d target = (m_overrideTarget != null)
            ? m_overrideTarget
            : m_turret.getAllianceTarget();
 
        // --- Shoot-on-the-move: project robot to its future position ---
        double rawDistanceMeters = robotPose.getTranslation().getDistance(target);
        double rawDistanceInches = Units.metersToInches(rawDistanceMeters);
        double airtime = ShooterTable.getAirtime(rawDistanceInches);
 
        ChassisSpeeds fieldSpeeds = m_drivetrain.getFieldRelativeSpeeds();
        Translation2d futurePosition = new Translation2d(
            robotPose.getX() + fieldSpeeds.vxMetersPerSecond * airtime,
            robotPose.getY() + fieldSpeeds.vyMetersPerSecond * airtime
        );
        Pose2d futurePose = new Pose2d(futurePosition, robotPose.getRotation());
        // ---------------------------------------------------------------
 
        double targetRotations = m_turret.calculateTargetEncoderPositionFromPose(futurePose, target);
        m_turret.setTargetPosition(targetRotations);
 
        SmartDashboard.putNumber("Turret/CommandTargetRotations", targetRotations);
        SmartDashboard.putNumber("Turret/RobotPoseX", robotPose.getX());
        SmartDashboard.putNumber("Turret/RobotPoseY", robotPose.getY());
        SmartDashboard.putNumber("Turret/RobotHeadingDeg", robotPose.getRotation().getDegrees());
        SmartDashboard.putNumber("Turret/Airtime", airtime);
        SmartDashboard.putNumber("Turret/FuturePoseX", futurePosition.getX());
        SmartDashboard.putNumber("Turret/FuturePoseY", futurePosition.getY());
    }
 
    @Override
    public void end(boolean interrupted) {
        m_turret.stopTurret();
        SmartDashboard.putBoolean("Turret/CommandRunning", false);
    }
 
    @Override
    public boolean isFinished() {
        return false;
    }
}