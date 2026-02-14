// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {

  private final TalonFX m_intakeMotor;
  private final TalonFXConfiguration m_intakeConfig;

  private final VelocityVoltage in_velRequest;


  /** Creates a new ExampleSubsystem. */
  public Intake() {
    m_intakeMotor = new TalonFX(Constants.intakeConstants.m_intakeMotor,Constants.intakeConstants.intakeCanbus);

    m_intakeConfig = new TalonFXConfiguration();
    m_intakeConfig.Slot0.kP = Constants.intakeConstants.k_intake_p;
    m_intakeConfig.Slot0.kI = Constants.intakeConstants.k_intake_i;
    m_intakeConfig.Slot0.kD = Constants.intakeConstants.k_intake_d;
    m_intakeConfig.Slot0.kS = Constants.intakeConstants.k_intake_s;
    m_intakeConfig.Slot0.kV = Constants.intakeConstants.k_intake_v;
    m_intakeConfig.Slot0.kA = Constants.intakeConstants.k_intake_a;
    m_intakeConfig.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;
    m_intakeConfig.MotionMagic.MotionMagicCruiseVelocity = Constants.intakeConstants.k_intake_velocity;
    m_intakeConfig.MotionMagic.MotionMagicAcceleration = Constants.intakeConstants.k_intake_acceleration;
    m_intakeConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    m_intakeConfig.CurrentLimits.StatorCurrentLimit = Constants.intakeConstants.k_currentLimit;

    in_velRequest = new VelocityVoltage(0);

    m_intakeMotor.getConfigurator().apply(m_intakeConfig);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public void runIntake(double velocity){
    m_intakeMotor.setControl(in_velRequest.withVelocity(velocity));
  }

  public void stopIntake(){
    m_intakeMotor.setControl(in_velRequest.withVelocity(0));
  }
}
