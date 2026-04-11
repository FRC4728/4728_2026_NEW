// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TurretShooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SetShooterCorner extends Command {
  TurretShooter turretshooter;

  // Called when the command is initially scheduled.
  public SetShooterCorner(TurretShooter turretshooter){
    this.turretshooter = turretshooter;
    addRequirements(turretshooter);
  }

  @Override
  public void initialize() {


  }

  @Override
  public void execute(){
    turretshooter.runHood(-11);
    turretshooter.runFlywheel(46.5);
  }

  @Override
  public void end(boolean interrupted) {

  }

  @Override
  public boolean isFinished(){
    return false;
  }
}