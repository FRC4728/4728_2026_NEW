// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
//import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Turret;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoAlignTurret extends Command {
  Turret m_turret;

  public AutoAlignTurret(Turret turret){
    m_turret = turret;
    addRequirements(m_turret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    NetworkTableInstance.getDefault().getTable("limelight-turret").getEntry("pipeline").setDouble(0);
  }

  @Override
  public void execute(){
    if(LimelightHelpers.getTV("limelight-turret")){
      m_turret.moveTurretVoltage(MathUtil.clamp(LimelightHelpers.getTX("limelight-turret")*Constants.TurretConstants.k_ll_kP,
      Constants.TurretConstants.k_turret_deadband_bottom,Constants.TurretConstants.k_turret_deadband_top));
    }
    else{
      DriverStation.reportError("No limelight target found!", false);
    }
  }

  @Override
  public void end(boolean interrupted) {
    m_turret.stopTurretVoltage();
  }

  @Override
  public boolean isFinished(){
    return true;
  }
}