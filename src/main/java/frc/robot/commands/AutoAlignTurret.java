package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.Turret;

public class AutoAlignTurret extends Command {

  private final Turret m_turret;

  // How close to a soft limit before we trigger a wrap (in rotor rotations)
  private static final double kWrapBuffer = 1.0;

  // Positions to jump to when wrapping (just inside the soft limits)
  private static final double kWrapToForward = 33.0; 
  private static final double kWrapToReverse = 1.0;  

  private boolean isWrapping = false;

  public AutoAlignTurret(Turret turret) {
    m_turret = turret;
    addRequirements(m_turret);
  }

  @Override
  public void initialize() {
    NetworkTableInstance.getDefault().getTable("limelight-turret").getEntry("pipeline").setDouble(0);
    isWrapping = false;
    System.out.println("AutoAlignTurret: command started");
  }

  @Override
  public void execute() {
    boolean hasTarget = LimelightHelpers.getTV("limelight-turret");
    double tx = LimelightHelpers.getTX("limelight-turret");
    double position = m_turret.getTurretPosition();

    double voltage = MathUtil.clamp(
        -tx * Constants.TurretConstants.k_ll_kP,
        -Constants.TurretConstants.k_ll_maxVoltage,
        Constants.TurretConstants.k_ll_maxVoltage
    );

    // Check if we need to wrap
    boolean nearForwardLimit = position >= (Constants.TurretConstants.k_turret_forwardSoftLimit - kWrapBuffer);
    boolean nearReverseLimit = position <= (Constants.TurretConstants.k_turret_reverseSoftLimit + kWrapBuffer);
    boolean tryingToGoForward = voltage > 0;
    boolean tryingToGoReverse = voltage < 0;

    if (nearForwardLimit && tryingToGoForward) {
      isWrapping = true;
      m_turret.moveTurretPosition(kWrapToReverse);
    } else if (nearReverseLimit && tryingToGoReverse) {
      isWrapping = true;
      m_turret.moveTurretPosition(kWrapToForward);
    } else {
      isWrapping = false;
      if (hasTarget) {
        m_turret.moveTurretVoltage(voltage);
      } else {
        m_turret.stopTurretVoltage();
      }
    }

    SmartDashboard.putBoolean("Turret/CommandRunning", true);
    SmartDashboard.putBoolean("Turret/CommandHasTarget", hasTarget);
    SmartDashboard.putBoolean("Turret/IsWrapping", isWrapping);
    SmartDashboard.putNumber("Turret/CommandVoltage", isWrapping ? 0 : voltage);
  }

  @Override
  public void end(boolean interrupted) {
    m_turret.stopTurretVoltage();
    isWrapping = false;
    SmartDashboard.putBoolean("Turret/CommandRunning", false);
    SmartDashboard.putNumber("Turret/CommandVoltage", 0);
    System.out.println("AutoAlignTurret: command ended, interrupted=" + interrupted);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}