package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.TurretShooter;

// Pass command that is used to shoot across the field.
// Flywheel and hood are now set dynamically based on distance to the pass target,
// with shoot-on-the-move compensation via PassTable.
public class Pass extends SequentialCommandGroup {
    public Pass(Indexer indexer, Kicker kicker, TurretShooter shooter, Turret turret,
                CommandSwerveDrivetrain drivetrain, Supplier<Translation2d> passTargetSupplier) {
        addCommands(
            new ParallelCommandGroup(
                new AutoAlignTurret(turret, drivetrain, passTargetSupplier),
                new SetShooterByPassDistance(shooter, drivetrain, passTargetSupplier),
                new SequentialCommandGroup(
                    new WaitCommand(0.2),
                    new ParallelCommandGroup(
                        new RunSpindexer(indexer),
                        new RunKickerUp(kicker)
                    )
                )
            )
        );
    }
}