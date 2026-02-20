package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
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

  private final MotionMagicVoltage mmRequest;
  private final VoltageOut voltReq;

  private final InterpolatingDoubleTreeMap m_angleOffsetTable = new InterpolatingDoubleTreeMap();
  private final InterpolatingDoubleTreeMap m_shooterRPMTable  = new InterpolatingDoubleTreeMap();

  public Turret() {
    limelight = new LimelightIO("limelight-turret");

    m_turretMotor = new TalonFX(
        Constants.TurretConstants.m_turretMotorId,
        Constants.TurretConstants.ringGearCanbus);

    m_turretConfig = new TalonFXConfiguration();

    m_turretConfig.Slot0.kP = Constants.TurretConstants.k_turret_p;
    m_turretConfig.Slot0.kI = Constants.TurretConstants.k_turret_i;
    m_turretConfig.Slot0.kD = Constants.TurretConstants.k_turret_d;
    m_turretConfig.Slot0.kS = Constants.TurretConstants.k_turret_s;
    m_turretConfig.Slot0.kV = Constants.TurretConstants.k_turret_v;
    m_turretConfig.Slot0.kA = Constants.TurretConstants.k_turret_a;
    m_turretConfig.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;

    m_turretConfig.MotionMagic.MotionMagicCruiseVelocity = Constants.TurretConstants.k_turret_velocity;
    m_turretConfig.MotionMagic.MotionMagicAcceleration   = Constants.TurretConstants.k_turret_acceleration;

    m_turretConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    m_turretConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    m_turretConfig.Feedback.RotorToSensorRatio   = Constants.TurretConstants.k_turret_gearRatio;

    m_turretConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable    = true;
    m_turretConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Constants.TurretConstants.k_maxRotations;
    m_turretConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable    = true;
    m_turretConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Constants.TurretConstants.k_minRotations;

    mmRequest = new MotionMagicVoltage(0).withSlot(0);
    voltReq   = new VoltageOut(0);

    // Angle offset LUT: distance (m) -> degrees to offset from TX
    m_angleOffsetTable.put(1.5, 0.0);
    m_angleOffsetTable.put(2.0, 0.0);
    m_angleOffsetTable.put(2.5, 0.0);
    m_angleOffsetTable.put(3.0, 0.0);
    m_angleOffsetTable.put(3.5, 0.0);
    m_angleOffsetTable.put(4.0, 0.0);
    m_angleOffsetTable.put(4.5, 0.0);
    m_angleOffsetTable.put(5.0, 0.0);

    // Shooter RPM LUT: distance (m) -> flywheel RPM
    m_shooterRPMTable.put(1.5, 2000.0);
    m_shooterRPMTable.put(2.0, 2400.0);
    m_shooterRPMTable.put(2.5, 2800.0);
    m_shooterRPMTable.put(3.0, 3200.0);
    m_shooterRPMTable.put(3.5, 3600.0);
    m_shooterRPMTable.put(4.0, 3900.0);
    m_shooterRPMTable.put(4.5, 4200.0);
    m_shooterRPMTable.put(5.0, 4500.0);

    try {
      m_turretMotor.getConfigurator().apply(m_turretConfig);
    } catch (Exception e) {
      DriverStation.reportWarning("Failed to configure Turret motor: " + e.toString(), true);
    }
  }

  // ---------------------------------------------------------------------------
  // HOMING
  // ---------------------------------------------------------------------------

  /**
   * Zeros the CANcoder at the current position.
   * Call this on robot enable while the turret is resting on its hard stop.
   * After calling this, position 0.0 = hard stop, 0.75 = 270 degrees of travel.
   * The turret center (forward-facing) will be at k_turretCenterRotations.
   */
  public void zeroTurretAtHardStop() {
    m_turretMotor.setPosition(0.0);
    SmartDashboard.putBoolean("Turret/Zeroed", true);
  }

  /**
   * Moves the turret to the center of its travel range (forward-facing position).
   * Call this after zeroing to confirm the center position is correct.
   */
  public Command homeTurretCommand() {
    return runOnce(() -> setTurretPosition(Constants.TurretConstants.k_turretCenterRotations))
        .withName("HomeTurret");
  }

  // ---------------------------------------------------------------------------
  // DISTANCE CALCULATION
  // ---------------------------------------------------------------------------

  public double getDistanceMeters() {
    if (!hasTarget()) return -1.0;

    double rawTY            = LimelightHelpers.getTY("limelight-turret");
    double correctedTY      = rawTY - Constants.TurretConstants.k_poiVerticalOffsetDeg;
    double angleToTargetRad = Math.toRadians(
        Constants.TurretConstants.k_limelightMountAngleDeg + correctedTY);

    return (Constants.TurretConstants.k_targetHeightMeters
        - Constants.TurretConstants.k_limelightHeightMeters)
        / Math.tan(angleToTargetRad);
  }

  // ---------------------------------------------------------------------------
  // POSITION CONTROL
  // ---------------------------------------------------------------------------

  public void setTurretPosition(double positionRotations) {
    double clamped = Math.max(
        Constants.TurretConstants.k_minRotations,
        Math.min(Constants.TurretConstants.k_maxRotations, positionRotations));
    m_turretMotor.setControl(mmRequest.withPosition(clamped));
  }

  // ---------------------------------------------------------------------------
  // HYBRID AIMING
  // ---------------------------------------------------------------------------

  public void aimWithCompensation(ChassisSpeeds fieldRelSpeeds) {
    if (!hasTarget()) return;

    double tx             = LimelightHelpers.getTX("limelight-turret");
    double distanceMeters = getDistanceMeters();
    if (distanceMeters < 0) return;

    double lutAngleOffsetDeg = m_angleOffsetTable.get(distanceMeters);
    double flightTimeSec     = distanceMeters / Constants.TurretConstants.k_fuelExitVelocityMPS;

    double turretHeadingDeg = getTurretPositionRotations() * 360.0;
    double txFieldRad       = Math.toRadians(tx + turretHeadingDeg);
    double targetX          = distanceMeters * Math.cos(txFieldRad);
    double targetY          = distanceMeters * Math.sin(txFieldRad);

    double adjustedX = targetX - (fieldRelSpeeds.vxMetersPerSecond * flightTimeSec);
    double adjustedY = targetY - (fieldRelSpeeds.vyMetersPerSecond * flightTimeSec);

    double physicsAngleDeg     = Math.toDegrees(Math.atan2(adjustedY, adjustedX));
    double finalAngleDeg       = physicsAngleDeg + lutAngleOffsetDeg;
    double finalAngleRotations = finalAngleDeg / 360.0;

    setTurretPosition(finalAngleRotations);

    SmartDashboard.putNumber("Turret/NaiveTXDeg",        tx);
    SmartDashboard.putNumber("Turret/LUTAngleOffsetDeg", lutAngleOffsetDeg);
    SmartDashboard.putNumber("Turret/PhysicsAngleDeg",   physicsAngleDeg);
    SmartDashboard.putNumber("Turret/FinalAngleDeg",     finalAngleDeg);
    SmartDashboard.putNumber("Turret/DistanceMeters",    distanceMeters);
    SmartDashboard.putNumber("Turret/FlightTimeMs",      flightTimeSec * 1000.0);
    SmartDashboard.putNumber("Turret/DesiredShooterRPM", getDesiredShooterRPM());
  }

  // ---------------------------------------------------------------------------
  // SHOOTER RPM QUERY
  // ---------------------------------------------------------------------------

  public double getDesiredShooterRPM() {
    double distance = getDistanceMeters();
    if (distance < 0) return -1.0;
    return m_shooterRPMTable.get(distance);
  }

  // ---------------------------------------------------------------------------
  // MANUAL CONTROL
  // ---------------------------------------------------------------------------

  public void moveTurretVoltage(double voltage) {
    m_turretMotor.setControl(voltReq.withOutput(voltage));
  }

  public void stopTurret() {
    m_turretMotor.setControl(voltReq.withOutput(0));
  }

  // ---------------------------------------------------------------------------
  // STATE QUERIES
  // ---------------------------------------------------------------------------

  public boolean hasTarget() {
    return LimelightHelpers.getTargetCount("limelight-turret") > 0;
  }

  public boolean isOnTarget() {
    if (!hasTarget()) return false;
    return Math.abs(LimelightHelpers.getTX("limelight-turret"))
        < Constants.TurretConstants.k_aimToleranceDegrees;
  }

  public double getTurretPositionRotations() {
    return m_turretMotor.getPosition().getValueAsDouble();
  }

  // ---------------------------------------------------------------------------
  // PERIODIC
  // ---------------------------------------------------------------------------

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Turret/TX",             LimelightHelpers.getTX("limelight-turret"));
    SmartDashboard.putNumber("Turret/TargetCount",    LimelightHelpers.getTargetCount("limelight-turret"));
    SmartDashboard.putBoolean("Turret/HasTarget",     hasTarget());
    SmartDashboard.putBoolean("Turret/OnTarget",      isOnTarget());
    SmartDashboard.putNumber("Turret/PositionRot",    getTurretPositionRotations());
    SmartDashboard.putNumber("Turret/DistanceMeters", getDistanceMeters());
  }

  // ---------------------------------------------------------------------------
  // COMMAND FACTORIES
  // ---------------------------------------------------------------------------

  public Command aimCommand(java.util.function.Supplier<ChassisSpeeds> speedsSupplier) {
    return run(() -> aimWithCompensation(speedsSupplier.get()))
        .withName("AimTurret_LUT+Compensation");
  }

  public Command stopCommand() {
    return runOnce(this::stopTurret).withName("StopTurret");
  }
}