package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.TurretShooter;
import frc.robot.subsystems.Turret;

public class Score extends SequentialCommandGroup {
    public Score(Intake intake, Indexer indexer, Kicker kicker, TurretShooter shooter, Turret turret) {
        addCommands(
            // Step 1: Wait until limelight sees target and turret is aligned
            new WaitUntilCommand(() ->
                LimelightHelpers.getTV("limelight-turret") && turret.isAligned()
            ),

            // Step 2: Shooter spins up, then feeding starts after 0.75s
            new ParallelCommandGroup(
                new RunShooter(shooter),
                new SequentialCommandGroup(
                    new WaitCommand(0.25),
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