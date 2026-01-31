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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Indexer extends SubsystemBase {

  private final TalonFX m_indexerMotor;
  private final TalonFXConfiguration m_indexerConfig;

  private final VelocityVoltage i_velRequest;


  /** Creates a new ExampleSubsystem. */
  public Indexer() {
    m_indexerMotor = new TalonFX(Constants.indexerConstants.m_indexerMotor,Constants.indexerConstants.indexerCanbus);
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

    i_velRequest = new VelocityVoltage(0.0);

    m_indexerMotor.getConfigurator().apply(m_indexerConfig);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Indexer Velocity",m_indexerMotor.getVelocity().getValueAsDouble());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public void runIndexer(double velocity){
    m_indexerMotor.setControl(i_velRequest.withVelocity(velocity));
  }

  public void stop(){
    m_indexerMotor.setControl(i_velRequest.withVelocity(0));
  }
}
