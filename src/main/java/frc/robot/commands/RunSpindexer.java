// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
//import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Indexer;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RunSpindexer extends Command {
  Indexer indexer;

  public RunSpindexer(Indexer indexer){
    this.indexer = indexer;
    addRequirements(indexer);
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  @Override
  public void execute(){
    //theoretically run indexer whatever direction it's supposed to go
    indexer.runIndexer(175);
  }

  @Override
  public void end(boolean interrupted) {
    indexer.stopIndexer();
  }

  @Override
  public boolean isFinished(){
    return false;
  }
}