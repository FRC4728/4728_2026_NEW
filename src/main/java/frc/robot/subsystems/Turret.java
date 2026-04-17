package frc.robot.subsystems;
 
import java.util.Optional;
 
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
 
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
 
public class Turret extends SubsystemBase {
    private final TalonFX m_turretMotor;
    private final VoltageOut voltReq;
    private final MotionMagicVoltage m_turretMotionMagic;
    private final NeutralOut m_brake;
 
    private double m_targetEncoderPosition = 0.0;
 
    public Turret() {
        m_turretMotor = new TalonFX(
            Constants.TurretConstants.m_turretMotorId,
            Constants.TurretConstants.ringGearCanbus
        );
 
        TalonFXConfiguration cfg = new TalonFXConfiguration();
 
        cfg.Slot0.kP = Constants.TurretConstants.k_turret_p;
        cfg.Slot0.kI = Constants.TurretConstants.k_turret_i;
        cfg.Slot0.kD = Constants.TurretConstants.k_turret_d;
        cfg.Slot0.kS = Constants.TurretConstants.k_turret_s;
        cfg.Slot0.kV = Constants.TurretConstants.k_turret_v;
        cfg.Slot0.kA = Constants.TurretConstants.k_turret_a;
        cfg.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;
 
        cfg.MotionMagic.MotionMagicCruiseVelocity = Constants.TurretConstants.k_turret_velocity;
        cfg.MotionMagic.MotionMagicAcceleration = Constants.TurretConstants.k_turret_acceleration;
        cfg.MotionMagic.MotionMagicJerk = Constants.TurretConstants.k_turret_jerk;
 
        cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        cfg.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
 
        cfg.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        cfg.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Constants.TurretConstants.k_turret_forwardSoftLimit;
        cfg.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        cfg.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Constants.TurretConstants.k_turret_reverseSoftLimit;
        cfg.CurrentLimits.StatorCurrentLimit = Constants.TurretConstants.k_turret_currentLimit;
 
        m_turretMotor.getConfigurator().apply(cfg);
 
        // Startup hard stop is mechanical 0 deg
        m_turretMotor.setPosition(0);
 
        m_turretMotionMagic = new MotionMagicVoltage(0).withSlot(0);
        voltReq = new VoltageOut(0);
        m_brake = new NeutralOut();
 
        // Enable LL Rewind via API
        LimelightHelpers.setRewindEnabled("limelight-left",true);
        LimelightHelpers.setRewindEnabled("limelight-right",true);
        
    }
 
    public double getTurretPosition() {
        return m_turretMotor.getPosition().getValueAsDouble();
    }
 
    public double getTargetPosition() {
        return m_targetEncoderPosition;
    }
 
    public void setTargetPosition(double encoderPosition) {
        double clamped = clampEncoderPosition(encoderPosition);
        if(Math.abs(clamped - m_targetEncoderPosition)<0.08){
            return;
        }
        m_targetEncoderPosition = clamped;
        m_turretMotor.setControl(m_turretMotionMagic.withPosition(m_targetEncoderPosition));
    }
 
    public boolean isAligned() {
        return Math.abs(getTurretPosition() - m_targetEncoderPosition)
            < Constants.PoseAimConstants.kTurretAlignToleranceMotorRotations;
    }
 
    public void moveTurretVoltage(double voltage) {
        m_turretMotor.setControl(voltReq.withOutput(voltage));
    }
 
    public void stopTurretVoltage() {
        m_turretMotor.setControl(voltReq.withOutput(0));
    }
 
    public void stopTurret() {
        m_turretMotor.setControl(m_brake);
    }
 
    public void moveTurretPosition(double position) {
        setTargetPosition(position);
    }
 
    public Translation2d getAllianceTarget() {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == Alliance.Red) {
            return Constants.FieldConstants.kRedScoringTarget;
        }
        return Constants.FieldConstants.kBlueScoringTarget;
    }
 
    public Translation2d getAlliancePassTarget() {
        return getAlliancePassTarget(1);
    }
 
    // Returns the pass target for the current alliance.
    // 1 = left, 2 = right
    public Translation2d getAlliancePassTarget(int option) {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        boolean isRed = alliance.isPresent() && alliance.get() == Alliance.Red;
        if (option == 2) {
            return isRed
                ? Constants.FieldConstants.kRedPassTarget2
                : Constants.FieldConstants.kBluePassTarget2;
        }
        return isRed
            ? Constants.FieldConstants.kRedPassTarget
            : Constants.FieldConstants.kBluePassTarget;
    }
 
    /**
     * Automatically selects a pass target based on the robot's current field-side position.
     * @param robotPose the current robot pose from odometry
     * @return the appropriate pass target Translation2d
     */
    public Translation2d getAutoPassTarget(Pose2d robotPose) {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        boolean isRed = alliance.isPresent() && alliance.get() == Alliance.Red;
 
        // Field Y midpoint (~4.1 m for a standard 2025/2026 field)
        final double kFieldMidY = 4.1;
 
        boolean robotOnLeftSide = robotPose.getY() >= kFieldMidY;
 
        // For Red alliance the "left" side in field coords is the right corner target
        int option;
        if (isRed) {
            option = robotOnLeftSide ? 2 : 1;
        } else {
            option = robotOnLeftSide ? 1 : 2;
        }
 
        SmartDashboard.putString("Pass/AutoTargetSide", robotOnLeftSide ? "Left" : "Right");
        SmartDashboard.putNumber("Pass/AutoTargetOption", option);
 
        return getAlliancePassTarget(option);
    }
 
    public Pose2d getTurretPose(Pose2d robotPose){
        return robotPose.transformBy(
            new Transform2d(
                new Translation2d(
                    Constants.PoseAimConstants.kTurretForwardOffsetMeters,
                    Constants.PoseAimConstants.kTurretLeftOffsetMeters
                ),
                new Rotation2d()
            )
        );
    }
 
    public double calculateTargetEncoderPositionFromPose(Pose2d robotPose) {
        return calculateTargetEncoderPositionFromPose(robotPose, getAllianceTarget());
    }
 
    public double calculateTargetEncoderPositionFromPose(Pose2d robotPose, Translation2d target) {
        Pose2d turretPose = getTurretPose(robotPose);
        Translation2d delta = target.minus(turretPose.getTranslation());
        Rotation2d fieldAngleToTarget = new Rotation2d(delta.getX(),delta.getY());//delta.getAngle();
 
        Rotation2d turretRobotRelative = fieldAngleToTarget.minus(robotPose.getRotation());
 
        double rearRelativeDeg = normalizeDegrees(turretRobotRelative.getDegrees() - 180);
 
        rearRelativeDeg = Math.max(-135.0, Math.min(135.0, rearRelativeDeg));
 
        double encoderDelta = rearRelativeDeg * Constants.PoseAimConstants.kEncoderUnitsPerTurretDegree;
 
        double desiredEncoderPosition = Constants.PoseAimConstants.kRearShotEncoderPosition + encoderDelta;
 
        SmartDashboard.putNumber("Expected Position",desiredEncoderPosition);
 
        return clampEncoderPosition(desiredEncoderPosition);
    }
 
    private double clampEncoderPosition(double encoderPosition) {
        return Math.max(
            Constants.PoseAimConstants.kTurretMinEncoderPosition,
            Math.min(Constants.PoseAimConstants.kTurretMaxEncoderPosition, encoderPosition)
        );
    }
 
    private static double normalizeDegrees(double deg){
        while (deg > 180.0) deg -= 360;
        while (deg < -180) deg += 360;
        return deg;
    }

 
    @Override
    public void periodic() {
        SmartDashboard.putNumber("Turret/Position", getTurretPosition());
        SmartDashboard.putNumber("Turret/TargetPosition", m_targetEncoderPosition);
        SmartDashboard.putBoolean("Turret/IsAligned", isAligned());
    }
}