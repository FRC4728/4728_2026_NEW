// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
//import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Intake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RunIntakeIn extends Command {
  Intake intake;
  
  public RunIntakeIn(Intake intake){
    this.intake = intake;
    addRequirements(intake);
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  @Override
  public void execute(){
    //theoretically run intake inwards??
    intake.runIntake(-75);
  }

  @Override
  public void end(boolean interrupted) {
    intake.stopIntake();
  }

  @Override
  public boolean isFinished(){
    return false;
  }
}