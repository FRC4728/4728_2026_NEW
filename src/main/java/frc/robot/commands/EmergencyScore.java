package frc.robot.commands;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.TurretShooter;


// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class EmergencyScore extends ParallelCommandGroup {
  
  public EmergencyScore(Intake intake, Indexer indexer, Kicker kicker, TurretShooter shooter, Turret turret, TurretShooter turretShooter) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    super(
      new RunShooter(shooter),
      new SetTurretCenter(turret),
      new SetHoodMid(turretShooter),
      new RunIntakeIn(intake),
      new RunSpindexer(indexer),
      new RunKickerUp(kicker));
  }
}