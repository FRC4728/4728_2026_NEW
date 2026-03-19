package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.Turret;
import frc.robot.Constants;

public class SearchForTarget extends Command {

    private final Turret m_turret;

    // Voltage to rotate while searching
    private static final double kSearchVoltage = 3.5;

    // Direction: 1.0 = positive, -1.0 = negative
    private double searchDirection = 1.0;

    public SearchForTarget(Turret turret) {
        m_turret = turret;
        addRequirements(m_turret);
    }

    @Override
    public void initialize() {
        searchDirection = 1.0;
    }

    @Override
    public void execute() {
        // If target found, stop — AutoAlignTurret will take over as default command
        if (LimelightHelpers.getTV("limelight-turret")) {
            m_turret.stopTurretVoltage();
            SmartDashboard.putBoolean("Turret/Searching", false);
            return;
        }

        double position = m_turret.getTurretPosition();

        // Reverse direction at soft limits
        if (position >= (Constants.TurretConstants.k_turret_forwardSoftLimit - 1.0)) {
            searchDirection = -1.0;
        } else if (position <= (Constants.TurretConstants.k_turret_reverseSoftLimit + 1.0)) {
            searchDirection = 1.0;
        }

        m_turret.moveTurretVoltage(kSearchVoltage * searchDirection);
        SmartDashboard.putBoolean("Turret/Searching", true);
    }

    @Override
    public void end(boolean interrupted) {
        m_turret.stopTurretVoltage();
        SmartDashboard.putBoolean("Turret/Searching", false);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}