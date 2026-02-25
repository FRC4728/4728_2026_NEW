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

  public AutoAlignTurret(Turret turret) {
    m_turret = turret;
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

    double voltage = MathUtil.clamp(
        tx * Constants.TurretConstants.k_ll_kP,
        -Constants.TurretConstants.k_ll_maxVoltage,
        Constants.TurretConstants.k_ll_maxVoltage
    );

    SmartDashboard.putBoolean("Turret/CommandRunning", true);
    SmartDashboard.putNumber("Turret/CommandVoltage", voltage);
    SmartDashboard.putBoolean("Turret/CommandHasTarget", hasTarget);

    if (hasTarget) {
      if (!m_turret.isAligned()) {
        m_turret.moveTurretVoltage(voltage);
      } else {
        m_turret.stopTurretVoltage();
      }
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