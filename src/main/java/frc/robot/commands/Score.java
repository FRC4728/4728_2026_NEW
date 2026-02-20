// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.*;


// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Score extends ParallelCommandGroup {
  /** Creates a new GoToL1. */
  public Score(Intake intake, Indexer indexer, Kicker kicker, TurretShooter shooter) {
    super(
      new RunIntakeIn(intake),
      new RunSpindexer(indexer),
      new RunKickerUp(kicker),
      new RunShooter(shooter)
      );
  }
}