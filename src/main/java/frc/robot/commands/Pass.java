package frc.robot.commands;
 
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.TurretShooter;
 
/**
 * Aims the turret at the alliance corner pass target, spins up the shooter
 * to pass settings, then feeds the ball.
 *
 * Bind with whileTrue() — turret and shooter return to their defaults on release.
 */
public class Pass extends SequentialCommandGroup {
    public Pass(Indexer indexer, Kicker kicker, TurretShooter shooter, Turret turret, CommandSwerveDrivetrain drivetrain)
     {
        addCommands(
            new ParallelCommandGroup(
                new AutoAlignTurret(turret, drivetrain, turret::getAlliancePassTarget),
                new SetShooterForPass(shooter),
                new SequentialCommandGroup(
                    new WaitCommand(0.4),
                    new ParallelCommandGroup(
                        new RunSpindexer(indexer),
                        new RunKickerUp(kicker)
                    )
                )
            )
        );
     }
    }                    
                
    
