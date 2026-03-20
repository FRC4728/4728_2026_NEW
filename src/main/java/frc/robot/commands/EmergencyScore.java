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

public class EmergencyScore extends SequentialCommandGroup {
    public EmergencyScore(Intake intake, Indexer indexer, Kicker kicker, TurretShooter shooter, Turret turret, TurretShooter turretShooter) {
        addCommands(

            // Step 2: Shooter spins up to distance-based setpoint, then feeding starts after 0.25s
            new ParallelCommandGroup(
                new RunShooter(shooter),
                new SetTurretCenter(turret),
                new SetHoodMid(turretShooter),
                new SequentialCommandGroup(
                    new WaitCommand(1),
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