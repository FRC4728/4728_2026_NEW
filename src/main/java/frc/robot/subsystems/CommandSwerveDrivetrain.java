package frc.robot.subsystems;
 
import static edu.wpi.first.units.Units.*;
 
import java.util.function.Supplier;
 
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
 
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
 
import frc.robot.LimelightHelpers; // adjust package if your helper lives elsewhere
import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;
 
/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements
 * Subsystem so it can easily be used in command-based projects.
 */
public class CommandSwerveDrivetrain extends TunerSwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
 
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;
 
    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean m_hasAppliedOperatorPerspective = false;
 
    /** Swerve request to apply during robot-centric path following */
    private final SwerveRequest.ApplyRobotSpeeds m_pathApplyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds();
 
    /* Swerve requests to apply during SysId characterization */
    private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization =
        new SwerveRequest.SysIdSwerveTranslation();
    private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization =
        new SwerveRequest.SysIdSwerveSteerGains();
    private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization =
        new SwerveRequest.SysIdSwerveRotation();
    
    private final Field2d m_field = new Field2d();
 
    /*
     * SysId routine for characterizing translation.
     * This is used to find PID gains for the drive motors.
     */
    private final SysIdRoutine m_sysIdRoutineTranslation = new SysIdRoutine(
        new SysIdRoutine.Config(
            Volts.of(0.2).div(Seconds.of(1)),
            Volts.of(4),
            null,
            state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> setControl(m_translationCharacterization.withVolts(output)),
            null,
            this
        )
    );
 
    /*
     * SysId routine for characterizing steer.
     * This is used to find PID gains for the steer motors.
     */
    private final SysIdRoutine m_sysIdRoutineSteer = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,
            Volts.of(7),
            null,
            state -> SignalLogger.writeString("SysIdSteer_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            volts -> setControl(m_steerCharacterization.withVolts(volts)),
            null,
            this
        )
    );
 
    /*
     * SysId routine for characterizing rotation.
     * This is used to find PID gains for the FieldCentricFacingAngle HeadingController.
     */
    private final SysIdRoutine m_sysIdRoutineRotation = new SysIdRoutine(
        new SysIdRoutine.Config(
            Volts.of(Math.PI / 6).per(Second),
            Volts.of(Math.PI),
            null,
            state -> SignalLogger.writeString("SysIdRotation_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> {
                setControl(m_rotationCharacterization.withRotationalRate(output.in(Volts)));
                SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
            },
            null,
            this
        )
    );
 
    private SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineTranslation;
 
    // ---------------- VISION CONFIG ----------------
    private static final String kRightLimelightName = "limelight-right";
    private static final String kleftlimelightname = "limelight-left";
 
    // Reject if spinning faster than this
    private static final double kMaxVisionOmegaDegPerSec = 30.0;
 
    // Reject if vision pose jumps more than this far from current odometry (meters)
    private static final double kMaxVisionPoseJumpMeters = 2.0;
 
    // Base stdDevs — scaled by distance in getVisionStdDevs()
    private static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.4, 0.4, 99999999);
    private static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(0.7, 0.7, 99999999);
 
    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     */
    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        configureAutoBuilder();
    }
 
    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     */
    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        double odometryUpdateFrequency,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, odometryUpdateFrequency, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        configureAutoBuilder();
    }
 
    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     */
    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        double odometryUpdateFrequency,
        Matrix<N3, N1> odometryStandardDeviation,
        Matrix<N3, N1> visionStandardDeviation,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, odometryUpdateFrequency, odometryStandardDeviation, visionStandardDeviation, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        configureAutoBuilder();
    }
 
    public void configureAutoBuilder() {
        try {
 
            var config = RobotConfig.fromGUISettings();
            AutoBuilder.configure(
                () -> getState().Pose,
                this::resetPose,
                () -> getState().Speeds,
                (speeds, feedforwards) -> setControl(
                    m_pathApplyRobotSpeeds.withSpeeds(ChassisSpeeds.discretize(speeds, 0.020))
                        .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                        .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())
                ),
                new PPHolonomicDriveController(
                    new PIDConstants(10, 0, 0),
                    new PIDConstants(7, 0, 0)
                ),
                config,
                () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
                this
            );
        } catch (Exception ex) {
            DriverStation.reportError(
                "Failed to load PathPlanner config and configure AutoBuilder",
                ex.getStackTrace()
            );
        }
    }
 
    /**
     * Returns the robot's velocity in the field-relative frame (m/s).
     * getState().Speeds is robot-centric, so we rotate it by the current heading
     * to get field-relative vx/vy for shoot-on-the-move future pose projection.
     */
    public ChassisSpeeds getFieldRelativeSpeeds() {
        ChassisSpeeds robotSpeeds = getState().Speeds;
        Rotation2d heading = getState().Pose.getRotation();
        return ChassisSpeeds.fromRobotRelativeSpeeds(robotSpeeds, heading);
    }
 
    /**
     * Returns a command that applies the specified control request to this swerve drivetrain.
     */
    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }
 
    /**
     * Runs the SysId Quasistatic test in the given direction.
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.quasistatic(direction);
    }
 
    /**
     * Runs the SysId Dynamic test in the given direction.
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.dynamic(direction);
    }
 
    @Override
    public void periodic() {
        /*
         * Periodically try to apply the operator perspective.
         */
        if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent(allianceColor -> {
                setOperatorPerspectiveForward(
                    allianceColor == Alliance.Red
                        ? kRedAlliancePerspectiveRotation
                        : kBlueAlliancePerspectiveRotation
                );
                m_hasAppliedOperatorPerspective = true;
            });
        }
 
        // IMU Mode 0 = use robot orientation fed via SetRobotOrientation (standard MegaTag2 usage)
        LimelightHelpers.SetIMUMode(kRightLimelightName, 0);
        LimelightHelpers.SetIMUMode(kleftlimelightname, 0);
 
        updateVisionFromLimelight(kRightLimelightName);
        updateVisionFromLimelight(kleftlimelightname);
    
        SmartDashboard.putNumber("Drive/PoseX", getState().Pose.getX());
        SmartDashboard.putNumber("Drive/PoseY", getState().Pose.getY());
        SmartDashboard.putNumber("Drive/PoseHeadingDeg", getState().Pose.getRotation().getDegrees());
 
        m_field.setRobotPose(getState().Pose);
 
        SmartDashboard.putData(m_field);
 
        Pose2d robotPose = getState().Pose;
        Translation2d scoringTarget = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red
            ? frc.robot.Constants.FieldConstants.kRedScoringTarget
            : frc.robot.Constants.FieldConstants.kBlueScoringTarget;
        double rawDistanceMeters = robotPose.getTranslation().getDistance(scoringTarget);
        double rawDistanceInches = Units.metersToInches(rawDistanceMeters);
 
        SmartDashboard.putNumber("Distance to target", rawDistanceInches);
 
        SmartDashboard.putNumber("Match Time",DriverStation.getMatchTime());
    }
 
    private void updateVisionFromLimelight(String limelightName) {
        // into SetRobotOrientation creates a feedback loop that makes MegaTag2 worse.
        double pigeonDegrees = getPigeon2().getYaw().getValueAsDouble();
 
        double yawRateDegPerSec = Math.toDegrees(getState().Speeds.omegaRadiansPerSecond);
        LimelightHelpers.SetRobotOrientation(limelightName, pigeonDegrees, 0, 0, 0, 0, 0);
        LimelightHelpers.PoseEstimate estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);
 
        boolean reject = shouldRejectVision(estimate, yawRateDegPerSec);
 
        if (reject) {
            SmartDashboard.putBoolean("Vision/" + limelightName + "/Accepted", false);
            return;
        }
 
        SmartDashboard.putBoolean("Vision/" + limelightName + "/Accepted", true);
        SmartDashboard.putNumber("Vision/" + limelightName + "/AvgTagDist", estimate.avgTagDist);
        SmartDashboard.putNumber("Vision/" + limelightName + "/LatencyMs", estimate.latency);
 
        Matrix<N3, N1> stdDevs = getVisionStdDevs(estimate);
        addVisionMeasurement(estimate.pose, estimate.timestampSeconds, stdDevs);
    }
 
    private boolean shouldRejectVision(LimelightHelpers.PoseEstimate estimate, double yawRateDegPerSec) {
    if (estimate == null) return true;
    if (estimate.tagCount <= 0) return true;

    if (Math.abs(yawRateDegPerSec) > 30.0) return true;

    // Only gate on odometry distance if we're enabled
    if (DriverStation.isEnabled()) {
        Pose2d currentPose = getState().Pose;
        if (currentPose.getTranslation().getDistance(estimate.pose.getTranslation()) > 1.0) return true;
    }

    if (estimate.tagCount >= 2 && estimate.avgTagDist > 4.0) return true;

    if (estimate.tagCount == 1 && estimate.rawFiducials != null && estimate.rawFiducials.length == 1) {
        if (estimate.rawFiducials[0].ambiguity > 0.5) return true;
        if (estimate.rawFiducials[0].distToCamera > 3.0) return true;
    }

    return false;
}
 
    private Matrix<N3, N1> getVisionStdDevs(LimelightHelpers.PoseEstimate estimate) {
        // Scale trust by distance — farther tag = less trust (larger stdDev)
        // Floor at 0.5m to prevent division-by-zero / infinite trust at very close range
        double distanceScale = Math.max(estimate.avgTagDist, 0.5);
 
        if (estimate.tagCount >= 2) {
            return kMultiTagStdDevs.times(distanceScale);
        }
        return kSingleTagStdDevs.times(distanceScale);
    }
 
    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();
 
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;
 
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }
 
    /**
     * Adds a vision measurement to the Kalman Filter.
     */
    @Override
    public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds));
    }
 
    @Override
    public void addVisionMeasurement(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs
    ) {
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds), visionMeasurementStdDevs);
    }
}