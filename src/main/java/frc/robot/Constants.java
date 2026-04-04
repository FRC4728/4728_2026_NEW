package frc.robot;
 
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
 
/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
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
 
        public static final double k_turret_p = 10; //4.8, 50
        public static final double k_turret_i = 0.0; //0
        public static final double k_turret_d = 0.1; //0.1
        public static final double k_turret_s = 0.0675; //0.0675
        public static final double k_turret_v = 0.12; //0.12
        public static final double k_turret_a = 0.01; //0.01
        public static final double k_turret_acceleration = 120; //160
        public static final double k_turret_velocity = 60;//90
        public static final double k_turret_jerk = 0;
 
        public static final double k_turret_gearRatio = 51.02;
 
        public static final double k_turret_forwardSoftLimit = 35;
        public static final double k_turret_reverseSoftLimit = 0.5;
 
        public static final double k_flywheel_p = 0.2;
        public static final double k_flywheel_i = 0.0;
        public static final double k_flywheel_d = 0.0;
        public static final double k_flywheel_s = 0.17;
        public static final double k_flywheel_v = 0.133333;
        public static final double k_flywheel_a = 0.01;
        public static final double k_flywheel_acceleration = 100;
        public static final double k_flywheel_velocity = 100;
 
        public static final double k_hood_p = 3.75;
        public static final double k_hood_i = 0.0;
        public static final double k_hood_d = 0.1;
        public static final double k_hood_s = 0.125;
        public static final double k_hood_v = 0.12;
        public static final double k_hood_a = 0.01;
        public static final double k_hood_acceleration = 100;
        public static final double k_hood_velocity = 100;
        public static final double k_hood_forwardSoftLimit = -0.5;
        public static final double k_hood_reverseSoftLimit = -15;
    }
 
    public static final class FieldConstants {
        // Ground location of target in meters
        public static final Translation2d kBlueScoringTarget = new Translation2d(4.60, 4.03);
        public static final Translation2d kRedScoringTarget  = new Translation2d(11.89, 4.03);
 
        // Corner pass targets — tune X/Y to match your field
        public static final Translation2d kBluePassTarget = new Translation2d(0.4, 5.7);
        public static final Translation2d kRedPassTarget  = new Translation2d(16, 2);
    }
 
    public static final class PoseAimConstants {
        // Turret angle relative to robot-forward when raw motor position is zero. 
        // Value of encoder when the turret is facing forward.
        public static final double kRearShotEncoderPosition = 17.7;
 
        // Raw motor-position calibration for a turret that travels 270 degrees across the soft limits.
        public static final double kTurretTravelDegrees = 275.0;
        public static final double kTurretMotorRotationsAcrossTravel =
            TurretConstants.k_turret_forwardSoftLimit - TurretConstants.k_turret_reverseSoftLimit;
        public static final double kTurretMotorRotationsPerDegree =
            kTurretMotorRotationsAcrossTravel / kTurretTravelDegrees;
        public static final double kTurretMotorRotationsPerRevolution =
            kTurretMotorRotationsPerDegree * 360.0;
 
        public static final double kTurretMinEncoderPosition = TurretConstants.k_turret_reverseSoftLimit;
        public static final double kTurretMaxEncoderPosition = TurretConstants.k_turret_forwardSoftLimit;
 
        public static final double kEncoderUnitsPerTurretDegree = (kTurretMaxEncoderPosition - kTurretMinEncoderPosition) / kTurretTravelDegrees;
 
        // Alignment tolerance in raw turret motor rotations.
        public static final double kTurretAlignToleranceMotorRotations = 0.01; //0.1
 
        // Distance filter settings for pose-derived range.
        public static final double kShooterDistanceDeadbandInches = 1.0;
        public static final double kDistanceFilterAlpha = 0.80;
 
        // Pose-derived distance sanity limits.
        public static final double kMinDistanceInches = 10.0;
        public static final double kMaxDistanceInches = 300.0;
 
        public static final double kTurretForwardOffsetMeters = Units.inchesToMeters(-4.5);
        public static final double kTurretLeftOffsetMeters = Units.inchesToMeters(4.5);
    }
 
    public static class indexerConstants {
        public static final String indexerCanbus = "canivore2";
        public static final int m_indexerMotor = 31;
        public static final double k_indexer_p = 0.2;
        public static final double k_indexer_i = 0.0;
        public static final double k_indexer_d = 0.0;
        public static final double k_indexer_s = 0.27;
        public static final double k_indexer_v = 0.156;
        public static final double k_indexer_a = 0.01;
        public static final double k_indexer_acceleration = 15;
        public static final double k_indexer_velocity = 500;
        public static final double k_jam_current = 200.0;
        public static final double k_unjam_velocity = -100.0;
        public static final double k_unjam_duration = 1;
    }
 
    public static class intakeConstants {
        public static final String intakeCanbus = "canivore2";
        public static final int m_intakeMotor = 33;
        public static final double k_intake_p = 0.2;
        public static final double k_intake_i = 0.0;
        public static final double k_intake_d = 0.0;
        public static final double k_intake_s = 0.2;
        public static final double k_intake_v = 0.154;
        public static final double k_intake_a = 0.01;
        public static final double k_intake_acceleration = 100;
        public static final double k_intake_velocity = 100;
        public static final double k_currentLimit = 75;
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
        public static final double k_kicker_p = 0.2;
        public static final double k_kicker_i = 0.0;
        public static final double k_kicker_d = 0.0;
        public static final double k_kicker_s = 0.38;
        public static final double k_kicker_v = 0.136;
        public static final double k_kicker_a = 0.01;
        public static final double k_kicker_acceleration = 100;
        public static final double k_kicker_velocity = 100;
    }
 
    public static class CandleConstants {
        public static final int canID = 55;
    }
 
    public static final class PassConstants {
        // Tune these on the field for your pass shot distance/angle
        public static final double kPassFlywheelRPS  = 45.0;
        public static final double kPassHoodPosition = -15.0;
    }
}