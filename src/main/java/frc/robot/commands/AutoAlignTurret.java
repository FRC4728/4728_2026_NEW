package frc.robot.commands;
 
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
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
 
        double targetRotations = m_turret.calculateTargetEncoderPositionFromPose(robotPose, target);
        m_turret.setTargetPosition(targetRotations);
 
        SmartDashboard.putNumber("Turret/CommandTargetRotations", targetRotations);
        SmartDashboard.putNumber("Turret/RobotPoseX", robotPose.getX());
        SmartDashboard.putNumber("Turret/RobotPoseY", robotPose.getY());
        SmartDashboard.putNumber("Turret/RobotHeadingDeg", robotPose.getRotation().getDegrees());
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