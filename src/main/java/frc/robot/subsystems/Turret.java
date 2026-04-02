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

    private double m_targetMotorRotations = 0.0;

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

        var status = m_turretMotor.getConfigurator().apply(cfg);
        if (!status.isOK()) {
            DriverStation.reportWarning("Failed to configure Turret motor: " + status.toString(), false);
        }

        // Startup zero must be repeatable every match.
        m_turretMotor.setPosition(0);

        m_turretMotionMagic = new MotionMagicVoltage(0).withSlot(0);
        voltReq = new VoltageOut(0);
        m_brake = new NeutralOut();
    }

    public double getTurretPosition() {
        return m_turretMotor.getPosition().getValueAsDouble();
    }

    public double getTargetPosition() {
        return m_targetMotorRotations;
    }

    public void setTargetPosition(double motorRotations) {
        m_targetMotorRotations = clampToSoftLimits(motorRotations);
        m_turretMotor.setControl(m_turretMotionMagic.withPosition(m_targetMotorRotations));
    }

    public boolean isAligned() {
        return Math.abs(getTurretPosition() - m_targetMotorRotations)
            < Constants.PoseAimConstants.kTurretAlignToleranceMotorRotations;
    }

    public void moveTurretVoltage(double voltage) {
        m_turretMotor.setControl(voltReq.withOutput(voltage));
    }

    public void stopTurretVoltage() {
        m_turretMotor.setControl(voltReq.withOutput(0));
    }

    public void moveTurretPosition(Double position) {
        setTargetPosition(position);
    }

    public void stopTurret() {
        m_turretMotor.setControl(m_brake);
    }

    public double calculateTargetRotationsFromPose(Pose2d robotPose) {
        Translation2d target = getAllianceTarget();
        Translation2d delta = target.minus(robotPose.getTranslation());

        Rotation2d fieldAngleToTarget = new Rotation2d(delta.getX(), delta.getY());
        Rotation2d turretRobotRelative = fieldAngleToTarget.minus(robotPose.getRotation());

        double desiredTurretDegrees =
            turretRobotRelative.getDegrees() + Constants.PoseAimConstants.kTurretZeroOffsetDegrees;

        double desiredMotorRotations = desiredTurretDegrees
            * Constants.PoseAimConstants.kTurretMotorRotationsPerDegree;

        return chooseClosestLegalRotation(desiredMotorRotations);
    }

    public Translation2d getAllianceTarget() {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == Alliance.Red) {
            return Constants.FieldConstants.kRedScoringTarget;
        }
        return Constants.FieldConstants.kBlueScoringTarget;
    }

    private double clampToSoftLimits(double motorRotations) {
        return Math.max(
            Constants.TurretConstants.k_turret_reverseSoftLimit,
            Math.min(Constants.TurretConstants.k_turret_forwardSoftLimit, motorRotations)
        );
    }

    private double chooseClosestLegalRotation(double desiredMotorRotations) {
        double current = getTurretPosition();

        double best = clampToSoftLimits(desiredMotorRotations);
        double bestError = Math.abs(best - current);

        for (int k = -3; k <= 3; k++) {
            double candidate = desiredMotorRotations
                + (k * Constants.PoseAimConstants.kTurretMotorRotationsPerRevolution);
            if (candidate < Constants.TurretConstants.k_turret_reverseSoftLimit) continue;
            if (candidate > Constants.TurretConstants.k_turret_forwardSoftLimit) continue;

            double error = Math.abs(candidate - current);
            if (error < bestError) {
                best = candidate;
                bestError = error;
            }
        }

        return best;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Turret/Position", m_turretMotor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Turret/Velocity", m_turretMotor.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Turret/AppliedVoltage", m_turretMotor.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("Turret/TargetPosition", m_targetMotorRotations);
        SmartDashboard.putBoolean("Turret/IsAligned", isAligned());
    }
}
