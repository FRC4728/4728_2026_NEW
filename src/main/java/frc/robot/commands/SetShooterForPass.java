package frc.robot.commands;
 
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.TurretShooter;
 
/**
 * Runs the flywheel and hood at fixed pass-shot values from PassConstants.
 * Tune kPassFlywheelRPS and kPassHoodPosition in Constants.PassConstants.
 */
public class SetShooterForPass extends Command {
    private final TurretShooter shooter;
 
    public SetShooterForPass(TurretShooter shooter) {
        this.shooter = shooter;
        addRequirements(shooter);
    }
 
    @Override
    public void execute() {
        shooter.runFlywheel(Constants.PassConstants.kPassFlywheelRPS);
        shooter.runHood(Constants.PassConstants.kPassHoodPosition);
    }
 
    @Override
    public void end(boolean interrupted) {
        shooter.coastFlywheel();
    }
 
    @Override
    public boolean isFinished() {
        return false;
    }
}