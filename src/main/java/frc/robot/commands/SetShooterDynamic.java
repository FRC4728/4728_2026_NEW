package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TurretShooter;

public class SetShooterDynamic extends Command {

    private final TurretShooter shooter;

    public SetShooterDynamic(TurretShooter shooter) {
        this.shooter = shooter;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        shooter.runFlywheelDyn();
        shooter.runHoodDyn();
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