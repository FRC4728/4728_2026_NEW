// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class TurretConstants {
    public static String turretCanbus = "rio";
    public static int m_turretMotorId = 50;
    public static int m_flywheelMotor1 = 51;
    public static int m_flywheelMotor2 = 52;
    public static int m_hoodMotor = 53;

    public static double k_turret_p = 0.1;
    public static double k_turret_i = 0.0;
    public static double k_turret_d = 0.0;
    public static double k_turret_s = 0.0;
    public static double k_turret_v = 0.0;
    public static double k_turret_a = 0.0;
    public static double k_turret_acceleration = 100;
    public static double k_turret_velocity = 100;

    public static double k_flywheel_p = 0.1;
    public static double k_flywheel_i = 0.0;
    public static double k_flywheel_d = 0.0;
    public static double k_flywheel_s = 0.0;
    public static double k_flywheel_v = 0.0;
    public static double k_flywheel_a = 0.0;
    public static double k_flywheel_acceleration = 100;
    public static double k_flywheel_velocity = 100;

    public static double k_hood_p = 0.1;
    public static double k_hood_i = 0.0;
    public static double k_hood_d = 0.0;
    public static double k_hood_s = 0.0;
    public static double k_hood_v = 0.0;
    public static double k_hood_a = 0.0;
    public static double k_hood_acceleration = 100;
    public static double k_hood_velocity = 100;

  }

  public static class indexerConstants {
    public static String indexerCanbus = "canivore";
    public static int m_indexerMotor = 54;
    public static double k_indexer_p = 0.1;
    public static double k_indexer_i = 0.0;
    public static double k_indexer_d = 0.0;
    public static double k_indexer_s = 0.0;
    public static double k_indexer_v = 0.0;
    public static double k_indexer_a = 0.0;
    public static double k_indexer_acceleration = 100;
    public static double k_indexer_velocity = 100;
  }

  public static class intakeConstants {
    public static String intakeCanbus = "canivore";
    public static int m_intakeMotor = 55;
    public static double k_intake_p = 0.1;
    public static double k_intake_i = 0.0;
    public static double k_intake_d = 0.0;
    public static double k_intake_s = 0.0;
    public static double k_intake_v = 0.0;
    public static double k_intake_a = 0.0;
    public static double k_intake_acceleration = 100;
    public static double k_intake_velocity = 100;

  }

  public static class ClimberConstants {
    public static String climberCanbus = "canivore";
    public static int m_climberMotor = 56; 
    public static double k_climber_p = 0.1;
    public static double k_climber_i = 0.0;
    public static double k_climber_d = 0.0;
    public static double k_climber_s = 0.0;
    public static double k_climber_v = 0.0;
    public static double k_climber_a = 0.0;
    public static double k_climber_acceleration = 100;
    public static double k_climber_velocity = 100;
  }

  public static class KickerConstants {
    public static String kickerCanbus = "canivore"; 
    public static int m_kickerMotor = 57;
    public static double k_kicker_p = 0.1;
    public static double k_kicker_i = 0.0;
    public static double k_kicker_d = 0.0;
    public static double k_kicker_s = 0.0;
    public static double k_kicker_v = 0.0;
    public static double k_kicker_a = 0.0;
    public static double k_kicker_acceleration = 100;
    public static double k_kicker_velocity = 100;
  }
}
