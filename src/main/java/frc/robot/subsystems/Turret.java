// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;

public class Turret extends SubsystemBase {

  private final TalonFX m_turretMotor;
  private final VoltageOut voltReq;

  public Turret() {
    m_turretMotor = new TalonFX(Constants.TurretConstants.m_turretMotorId, Constants.TurretConstants.ringGearCanbus);

    TalonFXConfiguration cfg = new TalonFXConfiguration();

    // PID / feedforward gains
    cfg.Slot0.kP = Constants.TurretConstants.k_turret_p;
    cfg.Slot0.kI = Constants.TurretConstants.k_turret_i;
    cfg.Slot0.kD = Constants.TurretConstants.k_turret_d;
    cfg.Slot0.kS = Constants.TurretConstants.k_turret_s;
    cfg.Slot0.kV = Constants.TurretConstants.k_turret_v;
    cfg.Slot0.kA = Constants.TurretConstants.k_turret_a;
    cfg.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;

    // Motion magic
    cfg.MotionMagic.MotionMagicCruiseVelocity = Constants.TurretConstants.k_turret_velocity;
    cfg.MotionMagic.MotionMagicAcceleration   = Constants.TurretConstants.k_turret_acceleration;

    // Rotor sensor with gear ratio applied
    cfg.Feedback.SensorToMechanismRatio = Constants.TurretConstants.k_turret_gearRatio;

    // Neutral mode
    cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    // Soft limits — 5 degrees of buffer on each end of the 270 degree range
    cfg.SoftwareLimitSwitch.ForwardSoftLimitEnable    = true;
    cfg.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Constants.TurretConstants.k_turret_forwardSoftLimit;
    cfg.SoftwareLimitSwitch.ReverseSoftLimitEnable    = true;
    cfg.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Constants.TurretConstants.k_turret_reverseSoftLimit;

    var status = m_turretMotor.getConfigurator().apply(cfg);
    if (!status.isOK()) {
      DriverStation.reportWarning("Failed to configure Turret motor: " + status.toString(), false);
    }

    // Zero the encoder at startup — robot must be placed against the hard stop before enabling
    m_turretMotor.setPosition(0);

    voltReq = new VoltageOut(0);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Turret/Position",       m_turretMotor.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("Turret/Velocity",       m_turretMotor.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Turret/AppliedVoltage", m_turretMotor.getMotorVoltage().getValueAsDouble());
    SmartDashboard.putNumber("Turret/LL_tX",          LimelightHelpers.getTX("limelight-turret"));
    SmartDashboard.putNumber("Turret/LL_Targets",     LimelightHelpers.getTargetCount("limelight-turret"));
    SmartDashboard.putBoolean("Turret/HasTarget",     LimelightHelpers.getTV("limelight-turret"));
    SmartDashboard.putBoolean("Turret/IsAligned",     isAligned());
  }

  /**
   * Returns true when the Limelight sees a target and tX is within the alignment deadband.
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