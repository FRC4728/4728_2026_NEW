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
    public static final String turretCanbus = "rio";
    public static final String ringGearCanbus = "canivore2";
    public static final int m_turretMotorId = 30;
    public static final int m_flywheelMotor1 = 34;
    public static final int m_flywheelMotor2 = 35;
    public static final int m_hoodMotor = 36;

    public static final double k_turret_p = 0.1;
    public static final double k_turret_i = 0.0;
    public static final double k_turret_d = 0.0;
    public static final double k_turret_s = 0.0;
    public static final double k_turret_v = 0.0;
    public static final double k_turret_a = 0.0;
    public static final double k_turret_acceleration = 100;
    public static final double k_turret_velocity = 100;
    public static final double k_ll_kP = 0.06;
    public static final double k_ll_maxVoltage = 6.0;       // max voltage output during auto-align (tune this)
    public static final double k_ll_alignDeadband = 1.5;    // degrees of tx within which turret is considered aligned
    public static final double k_turret_gearRatio = 51.0204081633;

    // Turret travel: 270 degrees of physical range, with 5 degrees of buffer on the forward end
    // Hard stop physically handles the reverse end so reverse limit is set to 0
    // 265 / 360 * 51.02 = 37.57 rotations
    public static final double k_turret_forwardSoftLimit = 37.57; // ~265 degrees
    public static final double k_turret_reverseSoftLimit = 0.0;   // hard stop handles this end
    

    public static final double k_flywheel_p = 0.38;
    public static final double k_flywheel_i = 0.0;
    public static final double k_flywheel_d = 0.0;
    public static final double k_flywheel_s = 0.17;
    public static final double k_flywheel_v = 0.133333; //2 X60 = 12/100*20/18*26 = 0.1333
    public static final double k_flywheel_a = 0.01;
    public static final double k_flywheel_acceleration = 100;
    public static final double k_flywheel_velocity = 100;

    public static final double k_hood_p = 0.1;
    public static final double k_hood_i = 0.0;
    public static final double k_hood_d = 0.0;
    public static final double k_hood_s = 0.0;
    public static final double k_hood_v = 0.0;
    public static final double k_hood_a = 0.0;
    public static final double k_hood_acceleration = 100;
    public static final double k_hood_velocity = 100;

  }

  public static class indexerConstants {
    public static final String indexerCanbus = "canivore2";
    public static final int m_indexerMotor = 31;
    public static final double k_indexer_p = 0.2;
    public static final double k_indexer_i = 0.0;
    public static final double k_indexer_d = 0.0;
    public static final double k_indexer_s = 0.27;
    public static final double k_indexer_v = 1.2; //X60 = 12/100Rps*5:1*2:1 =1.2
    public static final double k_indexer_a = 0.01;
    public static final double k_indexer_acceleration = 100;
    public static final double k_indexer_velocity = 500;
    public static final double k_jam_current = 100.0;  // stator amps above which indexer is considered jammed
    public static final double k_unjam_velocity = -25.0; // reverse velocity to clear jam
    public static final double k_unjam_duration = 0.25;   // seconds to run in reverse
  }

  public static class intakeConstants {
    public static final String intakeCanbus = "canivore2";
    public static final int m_intakeMotor = 33;
    public static final double k_intake_p = 0.35;
    public static final double k_intake_i = 0.0;
    public static final double k_intake_d = 0.0;
    public static final double k_intake_s = 0.2;
    public static final double k_intake_v = 0.56; // X60 = 12/100Rps*3:1*28:18 =0.56
    public static final double k_intake_a = 0.01;
    public static final double k_intake_acceleration = 100;
    public static final double k_intake_velocity = 100;
    public static final double k_currentLimit = 40;
  }

  public static class ClimberConstants {
    public static final String climberCanbus = "canivore2";
    public static final int m_climberMotor = 100;
    public static final double k_climber_p = 0.1;
    public static final double k_climber_i = 0.0;
    public static final double k_climber_d = 0.0;
    public static final double k_climber_s = 0.0;
    public static final double k_climber_v = 0.0;
    public static final double k_climber_a = 0.0;
    public static final double k_climber_acceleration = 100;
    public static final double k_climber_velocity = 100;
  }

  public static class KickerConstants {
    public static final String kickerCanbus = "canivore2";
    public static final int m_kickerMotor = 32;
    public static final double k_kicker_p = 0.50;
    public static final double k_kicker_i = 0.0;
    public static final double k_kicker_d = 0.0;
    public static final double k_kicker_s = 0.38;
    public static final double k_kicker_v = 0.164; //X44 = 12/125.5*36:21 =0.164
    public static final double k_kicker_a = 0.01;
    public static final double k_kicker_acceleration = 100;
    public static final double k_kicker_velocity = 100;
  }
}