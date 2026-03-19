package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.TurretShooter;
import frc.robot.subsystems.Turret;

public class Score extends SequentialCommandGroup {
    public Score(Indexer indexer, Kicker kicker, TurretShooter shooter, Turret turret) {
        addCommands(
            // Step 1: Wait until limelight sees target and turret is aligned
            new WaitUntilCommand(() ->
                LimelightHelpers.getTV("limelight-turret") && turret.isAligned()
            ),

            // Step 2: Shooter spins up to distance-based setpoint, then feeding starts after 0.25s
            new ParallelCommandGroup(
                new SetShooterByDistance(shooter),
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