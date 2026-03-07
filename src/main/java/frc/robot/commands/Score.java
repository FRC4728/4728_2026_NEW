package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.*;

public class Score extends SequentialCommandGroup {
    public Score(Intake intake, Indexer indexer, Kicker kicker, TurretShooter shooter, Turret turret) {
        addCommands(
            // Step 1: Wait until limelight sees target and turret is aligned
            new WaitUntilCommand(() ->
                LimelightHelpers.getTV("limelight-turret") && turret.isAligned()
            ),

            // Step 2: Start shooter and auto-align, wait for flywheel to spin up
            new ParallelCommandGroup(
                new RunShooter(shooter),
                new AutoAlignTurret(turret),
                new SequentialCommandGroup(
                    // Wait for shooter to reach speed
                    new WaitCommand(0.5),
                    // Then run everything else in parallel
                    new ParallelCommandGroup(
                        new RunIntakeIn(intake),
                        new RunSpindexer(indexer),
                        new RunKickerUp(kicker)
                    )
                )
            )
        );
    }
}