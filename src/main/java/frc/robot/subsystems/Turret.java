// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.ExternalFeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Turret extends SubsystemBase {

  private final TalonFXS m_turretMotor;
  private final TalonFX m_flywheelMotor1;
  private final TalonFX m_flywheelMotor2;
  private final TalonFX m_hoodMotor;

  private final TalonFXSConfiguration m_turretConfig;
  private final TalonFXConfiguration m_flywheelConfig, m_hoodConfig;

  /** Creates a new ExampleSubsystem. */
  public Turret() {
    m_turretMotor = new TalonFXS(Constants.TurretConstants.m_turretMotorId,Constants.TurretConstants.turretCanbus);
    m_flywheelMotor1 = new TalonFX(Constants.TurretConstants.m_flywheelMotor1,Constants.TurretConstants.turretCanbus);
    m_flywheelMotor2 = new TalonFX(Constants.TurretConstants.m_flywheelMotor2,Constants.TurretConstants.turretCanbus);
    m_hoodMotor = new TalonFX(Constants.TurretConstants.m_hoodMotor,Constants.TurretConstants.turretCanbus);
    
    //turret motor configurator
    m_turretConfig = new TalonFXSConfiguration();
    m_turretConfig.ExternalFeedback.ExternalFeedbackSensorSource = ExternalFeedbackSensorSourceValue.Commutation;
    m_turretConfig.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;
    m_turretConfig.Slot0.kP = Constants.TurretConstants.k_turret_p;
    m_turretConfig.Slot0.kI = Constants.TurretConstants.k_turret_i;
    m_turretConfig.Slot0.kD = Constants.TurretConstants.k_turret_d;
    m_turretConfig.Slot0.kS = Constants.TurretConstants.k_turret_s;
    m_turretConfig.Slot0.kV = Constants.TurretConstants.k_turret_v;
    m_turretConfig.Slot0.kA = Constants.TurretConstants.k_turret_a;
    m_turretConfig.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;
    m_turretConfig.MotionMagic.MotionMagicCruiseVelocity = Constants.TurretConstants.k_turret_velocity;
    m_turretConfig.MotionMagic.MotionMagicAcceleration = Constants.TurretConstants.k_turret_acceleration;
    m_turretConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    m_flywheelConfig = new TalonFXConfiguration();
    m_flywheelConfig.Slot0.kP = Constants.TurretConstants.k_turret_p;
    m_flywheelConfig.Slot0.kI = Constants.TurretConstants.k_turret_i;
    m_flywheelConfig.Slot0.kD = Constants.TurretConstants.k_turret_d;
    m_flywheelConfig.Slot0.kS = Constants.TurretConstants.k_turret_s;
    m_flywheelConfig.Slot0.kV = Constants.TurretConstants.k_turret_v;
    m_flywheelConfig.Slot0.kA = Constants.TurretConstants.k_turret_a;
    m_flywheelConfig.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;
    m_flywheelConfig.MotionMagic.MotionMagicCruiseVelocity = Constants.TurretConstants.k_turret_velocity;
    m_flywheelConfig.MotionMagic.MotionMagicAcceleration = Constants.TurretConstants.k_turret_acceleration;
    m_flywheelConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    m_hoodConfig = new TalonFXConfiguration();
    m_hoodConfig.Slot0.kP = Constants.TurretConstants.k_hood_p;
    m_hoodConfig.Slot0.kI = Constants.TurretConstants.k_hood_i;
    m_hoodConfig.Slot0.kD = Constants.TurretConstants.k_hood_d;
    m_hoodConfig.Slot0.kS = Constants.TurretConstants.k_hood_s;
    m_hoodConfig.Slot0.kV = Constants.TurretConstants.k_hood_v;
    m_hoodConfig.Slot0.kA = Constants.TurretConstants.k_hood_a;
    m_hoodConfig.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;
    m_hoodConfig.MotionMagic.MotionMagicCruiseVelocity = Constants.TurretConstants.k_hood_velocity;
    m_hoodConfig.MotionMagic.MotionMagicAcceleration = Constants.TurretConstants.k_hood_acceleration;
    m_hoodConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    try{
      m_turretMotor.getConfigurator().apply(m_turretConfig);
      m_flywheelMotor1.getConfigurator().apply(m_flywheelConfig);
      m_flywheelMotor2.getConfigurator().apply(m_flywheelConfig);
      m_hoodMotor.getConfigurator().apply(m_hoodConfig);
    }
    catch(Exception e1){
      DriverStation.reportWarning("Failed to configure Turret motor(s) "+e1.toString(),true);
    }

  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
