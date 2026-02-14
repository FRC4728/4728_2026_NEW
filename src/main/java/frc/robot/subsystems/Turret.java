// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.ExternalFeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightIO;

public class Turret extends SubsystemBase {

  private final TalonFX m_turretMotor;

  private final LimelightIO limelight;

  private final TalonFXConfiguration m_turretConfig;

  private final VelocityVoltage velRequest;
  private final VoltageOut voltReq;

  /** Creates a new ExampleSubsystem. */
  public Turret(){
    limelight = new LimelightIO("limelight-turret");

    m_turretMotor = new TalonFX(Constants.TurretConstants.m_turretMotorId,Constants.TurretConstants.ringGearCanbus);
    
    //turret motor configurator
    m_turretConfig = new TalonFXConfiguration();
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
    m_turretConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    m_turretConfig.Feedback.RotorToSensorRatio = Constants.TurretConstants.k_turret_gearRatio;

    velRequest = new VelocityVoltage(0).withSlot(0);
    voltReq = new VoltageOut(0);

   

    try{
      m_turretMotor.getConfigurator().apply(m_turretConfig);
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
    SmartDashboard.putNumber("Limelight tX",LimelightHelpers.getTX("limelight-turret"));
    SmartDashboard.putNumber("Limelight Targets",LimelightHelpers.getTargetCount("limelight-turret"));
    // This method will be called once per scheduler run
  }

  public void moveTurretVoltage(double voltage) {
      m_turretMotor.setControl(voltReq.withOutput(voltage));
  }
  public void stopTurretVoltage(){
    m_turretMotor.setControl(voltReq.withOutput(0));
  }


}
