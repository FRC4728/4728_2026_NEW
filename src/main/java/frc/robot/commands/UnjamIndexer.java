// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
 
package frc.robot.commands;
 
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Indexer;
 
public class UnjamIndexer extends SequentialCommandGroup {
 
  public UnjamIndexer(Indexer indexer) {
    addCommands(
      // Step 1: run in reverse for k_unjam_duration seconds to clear the jam
      new RunCommand(
        () -> indexer.runIndexer(Constants.indexerConstants.k_unjam_velocity), indexer)
        .withTimeout(Constants.indexerConstants.k_unjam_duration),
 
      // Step 2: stop the indexer after unjam
      new InstantCommand(() -> indexer.coastIndexer(), indexer)
    );
 
    // Explicitly declare requirement so WPILib knows this command
    // needs the indexer and can cancel whatever is currently using it.
    // NOTE: for this to work, your shooting command must be bound in
    // RobotContainer with .withInterruptBehavior(Command.InterruptionBehavior.kCancelSelf)
    // Example:
    //   shootButton.whileTrue(
    //       new ShootCommand(shooter, indexer)
    //           .withInterruptBehavior(Command.InterruptionBehavior.kCancelSelf)
    //   );
    //   unjamButton.onTrue(new UnjamIndexer(indexer));
    addRequirements(indexer);
  }
}