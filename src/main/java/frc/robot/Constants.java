// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class TurretConstants {
    public static String turretCanbus = "rio";
    public static String ringGearCanbus = "canivore2";
    public static int m_turretMotorId = 30;
    public static int m_flywheelMotor1 = 34;
    public static int m_flywheelMotor2 = 35;
    public static int m_hoodMotor = 36;

    public static double k_turret_p = 0.1;
    public static double k_turret_i = 0.0;
    public static double k_turret_d = 0.0;
    public static double k_turret_s = 0.0;
    public static double k_turret_v = 0.0;
    public static double k_turret_a = 0.0;
    public static double k_turret_acceleration = 100;
    public static double k_turret_velocity = 100;
    public static double k_ll_kP = 0.02;
    public static double k_turret_deadband_top = 20;
    public static double k_turret_deadband_bottom = -20;
    public static double k_turret_gearRatio = 1;

    // Soft limits: 0.0 = hard stop, 0.25 = 90 degrees of travel
    // Update k_maxRotations to 0.75 and k_turretCenterRotations to 0.375 when 270 degree travel is ready
    public static double k_minRotations = 0.0;
    public static double k_maxRotations = 0.25;
    public static double k_turretCenterRotations = 0.125;

    // Aiming
    public static double k_aimToleranceDegrees = 1.5;

    // Limelight geometry -- measure on your robot!
    public static double k_limelightMountAngleDeg = 30.0;
    public static double k_limelightHeightMeters  = 0.50;
    public static double k_targetHeightMeters     = 2.11;

    // POI crosshair vertical offset -- check your LL pipeline config
    public static double k_poiVerticalOffsetDeg = -5.0;

    // Shooter physics
    public static double k_fuelExitVelocityMPS = 15.0;

    // Flywheel at-speed tolerance (RPM)
    public static double k_flywheel_atSpeedToleranceRPM = 100.0;

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

  // ALL CLASSES BELOW ARE EXACTLY AS ORIGINAL -- no changes
  public static class indexerConstants {
    public static String indexerCanbus = "canivore2";
    public static int m_indexerMotor = 31;
    public static double k_indexer_p = 0.2;
    public static double k_indexer_i = 0.0;
    public static double k_indexer_d = 0.0;
    public static double k_indexer_s = 0.0;
    public static double k_indexer_v = 0.125;
    public static double k_indexer_a = 0.0;
    public static double k_indexer_acceleration = 100;
    public static double k_indexer_velocity = 500;
  }

  public static class intakeConstants {
    public static String intakeCanbus = "canivore2";
    public static int m_intakeMotor = 33;
    public static double k_intake_p = 0.1;
    public static double k_intake_i = 0.0;
    public static double k_intake_d = 0.0;
    public static double k_intake_s = 0.0;
    public static double k_intake_v = 0.0;
    public static double k_intake_a = 0.0;
    public static double k_intake_acceleration = 100;
    public static double k_intake_velocity = 100;
    public static double k_currentLimit = 100;
  }

  public static class ClimberConstants {
    public static String climberCanbus = "canivore2";
    public static int m_climberMotor = 100;
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
    public static String kickerCanbus = "canivore2";
    public static int m_kickerMotor = 32;
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