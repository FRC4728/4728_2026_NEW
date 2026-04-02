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
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

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

        cfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        cfg.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        cfg.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        cfg.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Constants.TurretConstants.k_turret_forwardSoftLimit;
        cfg.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        cfg.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Constants.TurretConstants.k_turret_reverseSoftLimit;

        m_turretMotor.getConfigurator().apply(cfg);

        // Startup hard stop is mechanical 0 deg
        m_turretMotor.setPosition(0);

        m_turretMotionMagic = new MotionMagicVoltage(0).withSlot(0);
        voltReq = new VoltageOut(0);
        m_brake = new NeutralOut();
    }

    public double getTurretPosition() {
        return m_turretMotor.getPosition().getValueAsDouble();
    }

    public double getTargetPosition() {
        return m_targetEncoderPosition;
    }

    public void setTargetPosition(double encoderPosition) {
        m_targetEncoderPosition = clampEncoderPosition(encoderPosition);
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

    public double calculateTargetEncoderPositionFromPose(Pose2d robotPose) {
        Translation2d target = getAllianceTarget();
        Translation2d delta = target.minus(robotPose.getTranslation());

        Rotation2d fieldAngleToTarget = new Rotation2d(delta.getX(), delta.getY());
        Rotation2d turretRobotRelative = fieldAngleToTarget.minus(robotPose.getRotation());

        // 0 deg relative means target is straight out robot front.
        // We shoot out robot back, so anchor around the encoder position
        // where the turret is aimed straight out the back.
        double desiredRelativeDegreesFromRear = turretRobotRelative.getDegrees() - 180.0;

        double encoderDelta =
            desiredRelativeDegreesFromRear * Constants.PoseAimConstants.kEncoderUnitsPerTurretDegree;

        double desiredEncoderPosition =
            Constants.PoseAimConstants.kRearShotEncoderPosition + encoderDelta;

        return clampEncoderPosition(desiredEncoderPosition);
    }

    private double clampEncoderPosition(double encoderPosition) {
        return Math.max(
            Constants.PoseAimConstants.kTurretMinEncoderPosition,
            Math.min(Constants.PoseAimConstants.kTurretMaxEncoderPosition, encoderPosition)
        );
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Turret/Position", getTurretPosition());
        SmartDashboard.putNumber("Turret/TargetPosition", m_targetEncoderPosition);
        SmartDashboard.putBoolean("Turret/IsAligned", isAligned());
    }
}