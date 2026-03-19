// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import com.ctre.phoenix6.controls.NeutralOut;


public class Indexer extends SubsystemBase {

  private final TalonFX m_indexerMotor;
  private final TalonFXConfiguration m_indexerConfig;
  private final VelocityVoltage i_velRequest;
  private final NeutralOut m_coast = new NeutralOut();

  private double targetVel;

  public Indexer() {
    m_indexerMotor = new TalonFX(Constants.indexerConstants.m_indexerMotor, Constants.indexerConstants.indexerCanbus);
    m_indexerConfig = new TalonFXConfiguration();
    m_indexerConfig.Slot0.kP = Constants.indexerConstants.k_indexer_p;
    m_indexerConfig.Slot0.kI = Constants.indexerConstants.k_indexer_i;
    m_indexerConfig.Slot0.kD = Constants.indexerConstants.k_indexer_d;
    m_indexerConfig.Slot0.kS = Constants.indexerConstants.k_indexer_s;
    m_indexerConfig.Slot0.kV = Constants.indexerConstants.k_indexer_v;
    m_indexerConfig.Slot0.kA = Constants.indexerConstants.k_indexer_a;
    m_indexerConfig.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;
    m_indexerConfig.MotionMagic.MotionMagicCruiseVelocity = Constants.indexerConstants.k_indexer_velocity;
    m_indexerConfig.MotionMagic.MotionMagicAcceleration = Constants.indexerConstants.k_indexer_acceleration;
    m_indexerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    i_velRequest = new VelocityVoltage(0.0).withSlot(0);

    m_indexerMotor.getConfigurator().apply(m_indexerConfig);

    targetVel = 90; //75
    SmartDashboard.putNumber("InputIndexerVelocity", targetVel);
  }
  
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Indexer/Velocity", m_indexerMotor.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Indexer/Voltage",  m_indexerMotor.getMotorVoltage().getValueAsDouble());
    SmartDashboard.putBoolean("Indexer/IsJammed", isJammed());
  }

  public boolean isJammed() {
    return m_indexerMotor.getStatorCurrent().getValueAsDouble() > Constants.indexerConstants.k_jam_current;
  }

  public Trigger getJamTrigger() {
    return new Trigger(this::isJammed);
  }

  public void runIndexer(double velocity) {
    m_indexerMotor.setControl(i_velRequest.withVelocity(velocity));
  }

  public void runIndexerDyn() {
    targetVel = SmartDashboard.getNumber("InputIndexerVelocity",targetVel);
    m_indexerMotor.setControl(i_velRequest.withVelocity(targetVel));
  }

  public void coastIndexer() {
    m_indexerMotor.setControl(m_coast);
  }

  public void stopIndexer() {
    m_indexerMotor.setControl(i_velRequest.withVelocity(0));
  }
}
