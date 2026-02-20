package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Turret;

public class AutoAlignTurret extends Command {
  private final Turret m_turret;
  private final CommandSwerveDrivetrain m_drivetrain;

  public AutoAlignTurret(Turret turret, CommandSwerveDrivetrain drivetrain) {
    m_turret    = turret;
    m_drivetrain = drivetrain;
    addRequirements(m_turret);
  }

  @Override
  public void initialize() {
    NetworkTableInstance.getDefault()
        .getTable("limelight-turret").getEntry("pipeline").setDouble(0);
  }

  @Override
  public void execute() {
    m_turret.aimWithCompensation(m_drivetrain.getCurrentChassisSpeeds());
  }

  @Override
  public void end(boolean interrupted) {
    m_turret.stopTurret();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}