package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class TurretShooter extends SubsystemBase {

  private final TalonFX m_flywheelMotor1;
  private final TalonFX m_flywheelMotor2;
  private final TalonFX m_hoodMotor;

  private final TalonFXConfiguration m_flywheelConfig;
  private final TalonFXConfiguration m_hoodConfig;

  private final VelocityVoltage fly_velRequest;
  private final VelocityVoltage hood_velRequest;
  private final VoltageOut voltReq;

  public TurretShooter() {
    m_flywheelMotor1 = new TalonFX(Constants.TurretConstants.m_flywheelMotor1, Constants.TurretConstants.turretCanbus);
    m_flywheelMotor2 = new TalonFX(Constants.TurretConstants.m_flywheelMotor2, Constants.TurretConstants.turretCanbus);
    m_hoodMotor      = new TalonFX(Constants.TurretConstants.m_hoodMotor,      Constants.TurretConstants.turretCanbus);

    m_flywheelConfig = new TalonFXConfiguration();
    m_flywheelConfig.Slot0.kP = Constants.TurretConstants.k_flywheel_p;
    m_flywheelConfig.Slot0.kI = Constants.TurretConstants.k_flywheel_i;
    m_flywheelConfig.Slot0.kD = Constants.TurretConstants.k_flywheel_d;
    m_flywheelConfig.Slot0.kS = Constants.TurretConstants.k_flywheel_s;
    m_flywheelConfig.Slot0.kV = Constants.TurretConstants.k_flywheel_v;
    m_flywheelConfig.Slot0.kA = Constants.TurretConstants.k_flywheel_a;
    m_flywheelConfig.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;
    m_flywheelConfig.MotionMagic.MotionMagicCruiseVelocity = Constants.TurretConstants.k_flywheel_velocity;
    m_flywheelConfig.MotionMagic.MotionMagicAcceleration   = Constants.TurretConstants.k_flywheel_acceleration;
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
    m_hoodConfig.MotionMagic.MotionMagicAcceleration   = Constants.TurretConstants.k_hood_acceleration;
    m_hoodConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    fly_velRequest  = new VelocityVoltage(0).withSlot(0);
    hood_velRequest = new VelocityVoltage(0).withSlot(0);
    voltReq         = new VoltageOut(0);

    try {
      m_flywheelMotor1.getConfigurator().apply(m_flywheelConfig);
      m_flywheelMotor2.getConfigurator().apply(m_flywheelConfig);
      m_hoodMotor.getConfigurator().apply(m_hoodConfig);
    } catch (Exception e) {
      DriverStation.reportWarning("Failed to configure TurretShooter motors: " + e.toString(), true);
    }
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Shooter/Flywheel1RPM", m_flywheelMotor1.getVelocity().getValueAsDouble() * 60.0);
    SmartDashboard.putNumber("Shooter/Flywheel2RPM", m_flywheelMotor2.getVelocity().getValueAsDouble() * 60.0);
    SmartDashboard.putBoolean("Shooter/AtSpeed",     isAtSpeed());
  }

  public void runFlywheel(double velocityRPM) {
    double rotPerSec = velocityRPM / 60.0;
    m_flywheelMotor1.setControl(fly_velRequest.withVelocity(rotPerSec));
    m_flywheelMotor2.setControl(new Follower(Constants.TurretConstants.m_flywheelMotor1, MotorAlignmentValue.Aligned));
  }

  public void stopFlywheel() {
    m_flywheelMotor1.setControl(voltReq.withOutput(0));
    m_flywheelMotor2.setControl(voltReq.withOutput(0));
  }

  public void setHoodPosition(double positionRotations) {
    m_hoodMotor.setControl(hood_velRequest.withVelocity(positionRotations));
  }

  public boolean isAtSpeed() {
    double errorRPS = Math.abs(m_flywheelMotor1.getClosedLoopError().getValueAsDouble());
    return errorRPS < (Constants.TurretConstants.k_flywheel_atSpeedToleranceRPM / 60.0);
  }
}