package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import java.util.Optional;
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
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
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

    // Reject very fast spin; Limelight docs commonly gate vision while spinning hard
    private static final double kMaxVisionOmegaDegPerSec = 360.0;

    // Simple distance-based trust tuning
    private static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.7, 0.7, 9999999.0);
    private static final Matrix<N3, N1> kSingleTagStdDevsClose = VecBuilder.fill(1.5, 1.5, 9999999.0);

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

        if(LimelightHelpers.getTV(kRightLimelightName)){
            updateVisionFromLimelight(kRightLimelightName);
        }
        else if(LimelightHelpers.getTV(kleftlimelightname)){
            updateVisionFromLimelight(kleftlimelightname);
        }
        else {
            return;
        }
        
    
        SmartDashboard.putNumber("Drive/PoseX", getState().Pose.getX());
        SmartDashboard.putNumber("Drive/PoseY", getState().Pose.getY());
        SmartDashboard.putNumber("Drive/PoseHeadingDeg", getState().Pose.getRotation().getDegrees());
    }

    private void updateVisionFromLimelight(String limelightName) {
        // Tell LL our current robot orientation before requesting MegaTag2
        double yawDeg = getState().Pose.getRotation().getDegrees();
        double yawRateDegPerSec = Math.toDegrees(getState().Speeds.omegaRadiansPerSecond);
        LimelightHelpers.PoseEstimate estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);
        LimelightHelpers.SetRobotOrientation(
            limelightName,
            yawDeg,
            yawRateDegPerSec,
            0.0,
            0.0,
            0.0,
            0.0
        );

        boolean reject = shouldRejectVision(estimate, yawRateDegPerSec);

        SmartDashboard.putBoolean("Vision/" + limelightName + "/Rejected", reject);
        SmartDashboard.putNumber("Vision/" + limelightName + "/TagCount", estimate.tagCount);
        SmartDashboard.putNumber("Vision/" + limelightName + "/AvgTagDist", estimate.avgTagDist);
        SmartDashboard.putNumber("Vision/" + limelightName + "/LatencyMs", estimate.latency);

        if (reject) {
            return;
        }

        Matrix<N3, N1> stdDevs = getVisionStdDevs(estimate);
        addVisionMeasurement(estimate.pose, estimate.timestampSeconds, stdDevs);

        SmartDashboard.putBoolean("Vision/" + limelightName + "/Accepted", true);
        SmartDashboard.putNumber("Vision/" + limelightName + "/PoseX", estimate.pose.getX());
        SmartDashboard.putNumber("Vision/" + limelightName + "/PoseY", estimate.pose.getY());
        SmartDashboard.putNumber("Vision/" + limelightName + "/PoseHeadingDeg", estimate.pose.getRotation().getDegrees());
    }

    private boolean shouldRejectVision(LimelightHelpers.PoseEstimate estimate, double yawRateDegPerSec) {
        if (estimate == null) {
            return true;
        }

        if (estimate.tagCount <= 0) {
            return true;
        }

        if (Math.abs(yawRateDegPerSec) > kMaxVisionOmegaDegPerSec) {
            return true;
        }

        // Conservative single-tag rejection
        if (estimate.tagCount == 1 && estimate.rawFiducials != null && estimate.rawFiducials.length == 1) {
            if (estimate.rawFiducials[0].ambiguity > 0.7) {
                return true;
            }
            if (estimate.rawFiducials[0].distToCamera > 3.0) {
                return true;
            }
        }

        return false;
    }

    private Matrix<N3, N1> getVisionStdDevs(LimelightHelpers.PoseEstimate estimate) {
        if (estimate.tagCount >= 2) {
            return kMultiTagStdDevs;
        }
        return kSingleTagStdDevsClose;
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