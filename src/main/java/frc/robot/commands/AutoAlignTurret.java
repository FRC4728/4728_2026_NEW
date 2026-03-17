package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.ShooterTable;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Turret;

public class AutoAlignTurret extends Command {

  private final Turret m_turret;
  private final CommandSwerveDrivetrain m_drivetrain;

  // Tuning constant — how aggressively to lead the target based on lateral speed
  // Increase if shots land behind the target when moving, decrease if they land ahead
  private static final double kVelocityCompensationScale = 1.75;

  public AutoAlignTurret(Turret turret, CommandSwerveDrivetrain drivetrain) {
    m_turret = turret;
    m_drivetrain = drivetrain;
    addRequirements(m_turret);
  }

  @Override
  public void initialize() {
    NetworkTableInstance.getDefault().getTable("limelight-turret").getEntry("pipeline").setDouble(0);
    System.out.println("AutoAlignTurret: command started");
  }

  @Override
  public void execute() {
    boolean hasTarget = LimelightHelpers.getTV("limelight-turret");
    double tx = LimelightHelpers.getTX("limelight-turret");
    double ty = LimelightHelpers.getTY("limelight-turret");

    // --- Velocity compensation ---
    // Get robot-relative lateral (strafe) velocity
    double lateralVelocityMps = m_drivetrain.getState().Speeds.vyMetersPerSecond;
  
    // Estimate distance and flight time
    double distanceInches = ShooterTable.getDistanceInches(ty);
    double distanceMeters = distanceInches * 0.0254;

    // Rough flight time — assumes ~8.5 m/s average ball speed in air
    double flightTimeSeconds = distanceMeters / 10;

    // Convert lateral velocity to angle offset in degrees
    double velocityOffsetDegrees = Math.toDegrees(
        Math.atan2(lateralVelocityMps * flightTimeSeconds * kVelocityCompensationScale, distanceMeters)
    );

    // Apply compensation to tx
    double compensatedTx = tx - velocityOffsetDegrees;

    double voltage = MathUtil.clamp(
        -compensatedTx * Constants.TurretConstants.k_ll_kP,
        -Constants.TurretConstants.k_ll_maxVoltage,
        Constants.TurretConstants.k_ll_maxVoltage
    );

    SmartDashboard.putBoolean("Turret/CommandRunning", true);
    SmartDashboard.putBoolean("Turret/CommandHasTarget", hasTarget);
    SmartDashboard.putNumber("Turret/CommandVoltage", voltage);
    SmartDashboard.putNumber("Turret/RawTx", tx);
    SmartDashboard.putNumber("Turret/CompensatedTx", compensatedTx);
    SmartDashboard.putNumber("Turret/VelocityOffsetDeg", velocityOffsetDegrees);
    SmartDashboard.putNumber("Turret/LateralVelocityMps", lateralVelocityMps);

    if (hasTarget) {
        m_turret.moveTurretVoltage(voltage);
    } else {
        m_turret.stopTurretVoltage();
    }
  }

  @Override
  public void end(boolean interrupted) {
    m_turret.stopTurretVoltage();
    SmartDashboard.putBoolean("Turret/CommandRunning", false);
    SmartDashboard.putNumber("Turret/CommandVoltage", 0);
    System.out.println("AutoAlignTurret: command ended, interrupted=" + interrupted);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}