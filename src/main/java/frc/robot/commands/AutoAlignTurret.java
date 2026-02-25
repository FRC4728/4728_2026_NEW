// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.NetworkTableInstance;
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
    // Switch to the AprilTag/target pipeline
    NetworkTableInstance.getDefault().getTable("limelight-turret").getEntry("pipeline").setDouble(0);
  }

  @Override
  public void execute() {
    if (LimelightHelpers.getTV("limelight-turret")) {
      if (!m_turret.isAligned()) {
        double voltage = LimelightHelpers.getTX("limelight-turret") * Constants.TurretConstants.k_ll_kP;
        voltage = MathUtil.clamp(voltage,
            -Constants.TurretConstants.k_ll_maxVoltage,
            Constants.TurretConstants.k_ll_maxVoltage);
        m_turret.moveTurretVoltage(voltage);
      } else {
        // On target — hold still
        m_turret.stopTurretVoltage();
      }
    } else {
      // No target visible — don't move
      m_turret.stopTurretVoltage();
    }
  }

  @Override
  public void end(boolean interrupted) {
    m_turret.stopTurretVoltage();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}