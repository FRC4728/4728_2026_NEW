// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;

public class Turret extends SubsystemBase {

  private final TalonFX m_turretMotor;
  private final TalonFXConfiguration m_turretConfig;
  private final VoltageOut voltReq;

  public Turret() {
    m_turretMotor = new TalonFX(Constants.TurretConstants.m_turretMotorId, Constants.TurretConstants.ringGearCanbus);

    m_turretConfig = new TalonFXConfiguration();

    // Soft limits — 5 degrees of buffer on each end of the 270 degree range
    // Hard stop is at 0, forward limit is ~265 degrees, reverse limit is ~5 degrees
    m_turretConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    m_turretConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Constants.TurretConstants.k_turret_forwardSoftLimit;
    m_turretConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    m_turretConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Constants.TurretConstants.k_turret_reverseSoftLimit;

    // PID / feedforward gains
    m_turretConfig.Slot0.kP = Constants.TurretConstants.k_turret_p;
    m_turretConfig.Slot0.kI = Constants.TurretConstants.k_turret_i;
    m_turretConfig.Slot0.kD = Constants.TurretConstants.k_turret_d;
    m_turretConfig.Slot0.kS = Constants.TurretConstants.k_turret_s;
    m_turretConfig.Slot0.kV = Constants.TurretConstants.k_turret_v;
    m_turretConfig.Slot0.kA = Constants.TurretConstants.k_turret_a;
    m_turretConfig.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;

    // Motion magic
    m_turretConfig.MotionMagic.MotionMagicCruiseVelocity = Constants.TurretConstants.k_turret_velocity;
    m_turretConfig.MotionMagic.MotionMagicAcceleration = Constants.TurretConstants.k_turret_acceleration;

    // Use the built-in rotor sensor — no CANcoder needed
    m_turretConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    m_turretConfig.Feedback.SensorToMechanismRatio = Constants.TurretConstants.k_turret_gearRatio;

    m_turretConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    voltReq = new VoltageOut(0);

    try {
      m_turretMotor.getConfigurator().apply(m_turretConfig);
      // Zero the encoder at startup — robot must be placed against the hard stop before enabling
      m_turretMotor.setPosition(0);
    } catch (Exception e) {
      DriverStation.reportWarning("Failed to configure Turret motor: " + e.toString(), true);
    }
  }

  @Override
  public void periodic() {
    // Turret telemetry for Elastic dashboard
    SmartDashboard.putNumber("Turret/Position", m_turretMotor.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("Turret/Velocity", m_turretMotor.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Turret/AppliedVoltage", m_turretMotor.getMotorVoltage().getValueAsDouble());

    // Limelight telemetry
    SmartDashboard.putNumber("Turret/LL_tX", LimelightHelpers.getTX("limelight-turret"));
    SmartDashboard.putNumber("Turret/LL_Targets", LimelightHelpers.getTargetCount("limelight-turret"));
    SmartDashboard.putBoolean("Turret/HasTarget", LimelightHelpers.getTV("limelight-turret"));
    SmartDashboard.putBoolean("Turret/IsAligned", isAligned());
  }

  /**
   * Returns true when the Limelight tx is within the alignment deadband.
   * Use this to confirm the turret is on target before firing.
   */
  public boolean isAligned() {
    return LimelightHelpers.getTV("limelight-turret")
        && Math.abs(LimelightHelpers.getTX("limelight-turret")) < Constants.TurretConstants.k_ll_alignDeadband;
  }

  public void moveTurretVoltage(double voltage) {
    m_turretMotor.setControl(voltReq.withOutput(voltage));
  }

  public void stopTurretVoltage() {
    m_turretMotor.setControl(voltReq.withOutput(0));
  }
}