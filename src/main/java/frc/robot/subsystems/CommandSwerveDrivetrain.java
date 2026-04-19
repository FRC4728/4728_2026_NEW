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
 
import frc.robot.LimelightHelpers;
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
    private static final String kleftlimelightname  = "limelight-left";
    private static final String kBackLimelightName  = "limelight-back";

    //Camera pose offsets — sourced directly from each Limelight's web UI ──
    //setCameraPose_RobotSpace convention: forward(m), LEFT(m), up(m), roll(deg), pitch(deg), yaw(deg)

    //limelight-right: Forward -0.35, Right 0.3 → Left -0.3, Up 0.22, Pitch 31.2, Yaw -90
    private static final double kRightCamForwardM  = -0.35;
    private static final double kRightCamLeftM     = -0.30;  // negated from LL Right = 0.3
    private static final double kRightCamUpM       =  0.22;
    private static final double kRightCamRollDeg   =  0.0;
    private static final double kRightCamPitchDeg  =  31.2;
    private static final double kRightCamYawDeg    = -90.0;

    //limelight-left: Forward -0.35, Right -0.3 → Left 0.3, Up 0.22, Pitch 31.2, Yaw 90
    private static final double kLeftCamForwardM   = -0.35;
    private static final double kLeftCamLeftM      =  0.30;  // negated from LL Right = -0.3
    private static final double kLeftCamUpM        =  0.22;
    private static final double kLeftCamRollDeg    =  0.0;
    private static final double kLeftCamPitchDeg   =  31.2;
    private static final double kLeftCamYawDeg     =  90.0;

    //limelight-back: Forward -0.335, Right -0.064 → Left 0.064, Up 0.459, Pitch 21, Yaw 180
    private static final double kBackCamForwardM   = -0.335;
    private static final double kBackCamLeftM      =  0.064; // negated from LL Right = -0.064
    private static final double kBackCamUpM        =  0.459;
    private static final double kBackCamRollDeg    =  0.0;
    private static final double kBackCamPitchDeg   =  21.0;
    private static final double kBackCamYawDeg     =  180.0;

    //Post-pose-reset vision blackout ──
    //Blocks vision updates for this many seconds after any pose reset to prevent
    //stale pre-reset measurements from corrupting the new pose.
    private static final double kVisionBlackoutAfterResetSeconds = 0.15;
    private double m_lastPoseResetTimestamp = 0.0;

    //Pose convergence / "stable" counter
    private static final double kPoseAgreementToleranceMeters = 0.10;
    private static final int    kPoseStableThreshold = 50;
    private int m_consecutiveAgreeingVisionUpdates = 0;

    //prevents the stability counter from being zeroed by a later camera in the same periodic() loop after an earlier camera already accepted and incremented it.
    private boolean m_anyVisionAcceptedThisLoop = false;
 
    //Base stdDevs rotation column pinned to infinity so MT2 never corrects heading
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
        configureVision();
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
        configureVision();
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
        configureVision();
        if (Utils.isSimulation()) {
            startSimThread();
        }
        configureAutoBuilder();
    }

    /**
     * - Sets IMU mode 4 on all three Limelights so MT2 uses ONLY the heading we
     *   feed via SetRobotOrientation (our odometry pose heading) and ignores
     *   each Limelight's internal IMU entirely. This eliminates the startup-angle
     *   dependency that came from feeding raw Pigeon yaw.
     */
    private void configureVision() {
        // IMU mode 4 = use external yaw only (what we send via SetRobotOrientation).
        LimelightHelpers.SetIMUMode(kRightLimelightName, 4);
        LimelightHelpers.SetIMUMode(kleftlimelightname,  4);
        LimelightHelpers.SetIMUMode(kBackLimelightName,  4);

        //Push each camera's mounting offset to its Limelight so MT2 knows exactly where the camera sits on the robot when solving the field pose.
        LimelightHelpers.setCameraPose_RobotSpace(
            kRightLimelightName,
            kRightCamForwardM, kRightCamLeftM, kRightCamUpM,
            kRightCamRollDeg,  kRightCamPitchDeg, kRightCamYawDeg
        );
        LimelightHelpers.setCameraPose_RobotSpace(
            kleftlimelightname,
            kLeftCamForwardM, kLeftCamLeftM, kLeftCamUpM,
            kLeftCamRollDeg,  kLeftCamPitchDeg, kLeftCamYawDeg
        );
        LimelightHelpers.setCameraPose_RobotSpace(
            kBackLimelightName,
            kBackCamForwardM, kBackCamLeftM, kBackCamUpM,
            kBackCamRollDeg,  kBackCamPitchDeg, kBackCamYawDeg
        );
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
     * Override resetPose to record the timestamp so updateVisionFromLimelight
     * can ignore stale measurements captured before the reset.
     */
    @Override
    public void resetPose(Pose2d pose) {
        super.resetPose(pose);
        m_lastPoseResetTimestamp = Utils.getCurrentTimeSeconds();
    }

    
    //Call this whenever seedFieldCentric() is used
    public void markPoseReset() {
        m_lastPoseResetTimestamp = Utils.getCurrentTimeSeconds();
    }

    /**
     * Returns true once vision and odometry have agreed within
     * kPoseAgreementToleranceMeters for kPoseStableThreshold consecutive loops.
     */
    public boolean isPoseStable() {
        return m_consecutiveAgreeingVisionUpdates >= kPoseStableThreshold;
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
 
    
    //Returns a command that applies the specified control request to this swerve drivetrain.
    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }
 
    
    //Runs the SysId Quasistatic test in the given direction.
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.quasistatic(direction);
    }
 
    
    //Runs the SysId Dynamic test in the given direction.
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.dynamic(direction);
    }
 
    @Override
    public void periodic() {
        
        //Periodically try to apply the operator perspective.
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

        //IMU mode is now set once in configureVision()
        m_anyVisionAcceptedThisLoop = false;
        updateVisionFromLimelight(kRightLimelightName);
        updateVisionFromLimelight(kleftlimelightname);
        updateVisionFromLimelight(kBackLimelightName);
    
        SmartDashboard.putNumber("Drive/PoseX", getState().Pose.getX());
        SmartDashboard.putNumber("Drive/PoseY", getState().Pose.getY());
        SmartDashboard.putNumber("Drive/PoseHeadingDeg", getState().Pose.getRotation().getDegrees());

        //Publish pose-stability state for driver/LED feedback.
        SmartDashboard.putBoolean("Vision/PoseStable", isPoseStable());
        SmartDashboard.putNumber("Vision/StableCount", m_consecutiveAgreeingVisionUpdates);
 
        m_field.setRobotPose(getState().Pose);
        SmartDashboard.putData(m_field);
 
        Pose2d robotPose = getState().Pose;
        Translation2d scoringTarget = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red
            ? frc.robot.Constants.FieldConstants.kRedScoringTarget
            : frc.robot.Constants.FieldConstants.kBlueScoringTarget;
        double rawDistanceMeters = robotPose.getTranslation().getDistance(scoringTarget);
        double rawDistanceInches = Units.metersToInches(rawDistanceMeters);
 
        SmartDashboard.putNumber("Distance to target", rawDistanceInches);
        SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());
    }
 
    private void updateVisionFromLimelight(String limelightName) {
        //Skip vision updates briefly after any pose reset ──
        // Prevents measurements captured before the reset from being fused against
        // the new pose, which would cause a brief violent oscillation in the estimator.
        if (Utils.getCurrentTimeSeconds() - m_lastPoseResetTimestamp < kVisionBlackoutAfterResetSeconds) {
            return;
        }

        //Use odometry pose heading instead of raw Pigeon yaw
        double odometryYawDegrees = getState().Pose.getRotation().getDegrees();

        double yawRateDegPerSec = Math.toDegrees(getState().Speeds.omegaRadiansPerSecond);

        //Pass actual yaw rate as second argument
        // Previously this was hardcoded to 0. Passing the real rate lets the Limelight internally reduce trust during high-spin moments.
        LimelightHelpers.SetRobotOrientation(limelightName, odometryYawDegrees, yawRateDegPerSec, 0, 0, 0, 0);

        LimelightHelpers.PoseEstimate estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);
 
        boolean reject = shouldRejectVision(estimate, yawRateDegPerSec);
 
        if (reject) {
            SmartDashboard.putBoolean("Vision/" + limelightName + "/Accepted", false);
            //Only zero the stability counter if no other camera has already accepted
            //a measurement this loop. Without this guard, a rejection from camera B
            //would erase the increment that camera A just made in the same periodic().
            if (!m_anyVisionAcceptedThisLoop) {
                m_consecutiveAgreeingVisionUpdates = 0;
            }
            return;
        }
 
        SmartDashboard.putBoolean("Vision/" + limelightName + "/Accepted", true);
        SmartDashboard.putNumber("Vision/" + limelightName + "/AvgTagDist", estimate.avgTagDist);
        SmartDashboard.putNumber("Vision/" + limelightName + "/LatencyMs", estimate.latency);
 
        Matrix<N3, N1> stdDevs = getVisionStdDevs(estimate);
        addVisionMeasurement(estimate.pose, estimate.timestampSeconds, stdDevs);

        //Track how many consecutive updates vision agrees with odometry
        //Increment when the vision pose is close to where odometry thinks we are reset to 0 on any disagreement. isPoseStable() gates on this counter.
        double poseError = getState().Pose.getTranslation()
            .getDistance(estimate.pose.getTranslation());
        if (poseError < kPoseAgreementToleranceMeters) {
            m_consecutiveAgreeingVisionUpdates++;
            m_anyVisionAcceptedThisLoop = true;
        } else {
            m_consecutiveAgreeingVisionUpdates = 0;
        }
    }
 
    private boolean shouldRejectVision(LimelightHelpers.PoseEstimate estimate, double yawRateDegPerSec) {
        if (estimate == null) return true;
        if (estimate.tagCount <= 0) return true;

        if (Math.abs(yawRateDegPerSec) > 180.0) return true;

        //Only gate on odometry distance if we're enabled
        if (DriverStation.isEnabled()) {
            Pose2d currentPose = getState().Pose;
            if (currentPose.getTranslation().getDistance(estimate.pose.getTranslation()) > 3.0) return true;
        }

        if (estimate.tagCount >= 2 && estimate.avgTagDist > 4.0) return true;

        if (estimate.tagCount == 1 && estimate.rawFiducials != null && estimate.rawFiducials.length == 1) {
            if (estimate.rawFiducials[0].ambiguity > 0.5) return true;
            if (estimate.rawFiducials[0].distToCamera > 3.0) return true;
        }

        return false;
    }
 
    private Matrix<N3, N1> getVisionStdDevs(LimelightHelpers.PoseEstimate estimate) {
        if (estimate.tagCount >= 2) {
            return kMultiTagStdDevs;
        }
        return kSingleTagStdDevs;
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
 
    
    //Adds a vision measurement to the Kalman Filter.
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