// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
//import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.TurretShooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SetHoodMin extends Command {
  TurretShooter turretShooter;

  // Called when the command is initially scheduled.
  public SetHoodMin(TurretShooter turretShooter){
    this.turretShooter = turretShooter;
    addRequirements(turretShooter);
  }

  @Override
  public void initialize() {


  }

  @Override
  public void execute(){
    turretShooter.runHood(-0.5);

  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished(){
    return false;
  }
}