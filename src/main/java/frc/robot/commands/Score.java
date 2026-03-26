package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.TurretShooter;
import frc.robot.subsystems.Turret;
import com.ctre.phoenix6.swerve.SwerveRequest;

public class Score extends SequentialCommandGroup {
    public Score(Indexer indexer, Kicker kicker, TurretShooter shooter, Turret turret, CommandSwerveDrivetrain drivetrain) {

        SwerveRequest.SwerveDriveBrake xLock = new SwerveRequest.SwerveDriveBrake();

        addCommands(
            // Step 1: X-lock wheels and wait for turret alignment simultaneously
            new ParallelCommandGroup(
                drivetrain.applyRequest(() -> xLock).withTimeout(0.1),
                new WaitUntilCommand(() ->
                    LimelightHelpers.getTV("limelight-turret") && turret.isAligned()
                )
            ),
                        // Step 2: Shooter spins up to distance-based setpoint, then feeding starts after 0.25s
            new ParallelCommandGroup(
                new SetShooterByDistance(shooter),
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