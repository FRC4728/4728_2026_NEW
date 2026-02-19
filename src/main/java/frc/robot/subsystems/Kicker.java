// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Kicker extends SubsystemBase {

  private final TalonFX m_kickerMotor;
  private final TalonFXConfiguration m_kickerConfig;
  private final VelocityVoltage k_velRequest;


  /** Creates a new ExampleSubsystem. */
  public Kicker() {
    m_kickerMotor = new TalonFX(Constants.KickerConstants.m_kickerMotor,Constants.KickerConstants.kickerCanbus);

    m_kickerConfig = new TalonFXConfiguration();
    m_kickerConfig.Slot0.kP = Constants.KickerConstants.k_kicker_p;
    m_kickerConfig.Slot0.kI = Constants.KickerConstants.k_kicker_i;
    m_kickerConfig.Slot0.kD = Constants.KickerConstants.k_kicker_d;
    m_kickerConfig.Slot0.kS = Constants.KickerConstants.k_kicker_s;
    m_kickerConfig.Slot0.kV = Constants.KickerConstants.k_kicker_v;
    m_kickerConfig.Slot0.kA = Constants.KickerConstants.k_kicker_a;
    m_kickerConfig.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;
    m_kickerConfig.MotionMagic.MotionMagicCruiseVelocity = Constants.KickerConstants.k_kicker_velocity;
    m_kickerConfig.MotionMagic.MotionMagicAcceleration = Constants.KickerConstants.k_kicker_acceleration;
    m_kickerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    m_kickerMotor.getConfigurator().apply(m_kickerConfig);

    k_velRequest = new VelocityVoltage(0).withSlot(0);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void runKicker(double velocity){
    m_kickerMotor.setControl(k_velRequest.withVelocity(velocity));
  }
  public void stopKicker(){
    m_kickerMotor.setControl(k_velRequest.withVelocity(0));
  }
}
