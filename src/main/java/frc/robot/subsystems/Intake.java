// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {

  private final TalonFX m_intakeMotor;
  private final TalonFXConfiguration m_intakeConfig;


  /** Creates a new ExampleSubsystem. */
  public Intake() {
    m_intakeMotor = new TalonFX(Constants.intakeConstants.m_intakeMotor,Constants.intakeConstants.intakeCanbus);

    m_intakeConfig = new TalonFXConfiguration();
    m_intakeConfig.Slot0.kP = Constants.intakeConstants.k_intake_p;
    m_intakeConfig.Slot0.kI = Constants.intakeConstants.k_intake_i;
    m_intakeConfig.Slot0.kD = Constants.intakeConstants.k_intake_d;
    m_intakeConfig.Slot0.kS = Constants.intakeConstants.k_intake_s;
    m_intakeConfig.Slot0.kV = Constants.intakeConstants.k_intake_v;
    m_intakeConfig.Slot0.kA = Constants.intakeConstants.k_intake_a;
    m_intakeConfig.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;
    m_intakeConfig.MotionMagic.MotionMagicCruiseVelocity = Constants.intakeConstants.k_intake_velocity;
    m_intakeConfig.MotionMagic.MotionMagicAcceleration = Constants.intakeConstants.k_intake_acceleration;
    m_intakeConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    m_intakeMotor.getConfigurator().apply(m_intakeConfig);

  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
