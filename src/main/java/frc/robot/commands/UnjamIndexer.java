// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Indexer;

public class UnjamIndexer extends SequentialCommandGroup {

  public UnjamIndexer(Indexer indexer) {
    addCommands(
      // Step 1: run in reverse for 0.5 seconds to clear the jam
      new RunCommand(() -> indexer.runIndexer(Constants.indexerConstants.k_unjam_velocity), indexer)
        .withTimeout(Constants.indexerConstants.k_unjam_duration),

      // Step 2: resume running forward automatically
      new InstantCommand(() -> indexer.runIndexer(55), indexer)
    );
  }
}
