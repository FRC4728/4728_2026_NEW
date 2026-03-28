package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.TurretShooter;

public class Score extends SequentialCommandGroup {
    public Score(Indexer indexer, Kicker kicker, TurretShooter shooter, Turret turret, CommandSwerveDrivetrain drivetrain) {
        SwerveRequest.SwerveDriveBrake xLock = new SwerveRequest.SwerveDriveBrake();

        addCommands(
            new ParallelCommandGroup(
                drivetrain.applyRequest(() -> xLock).withTimeout(0.1),
                new WaitUntilCommand(turret::isAligned)
            ),
            new ParallelCommandGroup(
                new SetShooterByDistance(shooter, drivetrain, turret),
                new SequentialCommandGroup(
                    new WaitCommand(0.45),
                    new ParallelCommandGroup(
                        new RunSpindexer(indexer),
                        new RunKickerUp(kicker)
                    )
                )
            )
        );
    }
}
