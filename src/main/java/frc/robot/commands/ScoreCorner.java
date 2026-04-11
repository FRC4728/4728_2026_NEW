package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.TurretShooter;

public class ScoreCorner extends SequentialCommandGroup {
    public ScoreCorner(
        Indexer indexer,
        Kicker kicker,
        TurretShooter shooter,
        Turret turret
    ) {
        //SwerveRequest.SwerveDriveBrake xLock = new SwerveRequest.SwerveDriveBrake();

        addCommands(
            new ParallelCommandGroup(
                new SetShooterCorner(shooter),
                new SetTurretLeftCorner (turret),
                new SequentialCommandGroup(
                    new WaitCommand(0.25),
                    new ParallelCommandGroup(
                        new RunSpindexer(indexer),
                        new RunKickerUp(kicker)
                    )
                )
            )
        );
    }
}
