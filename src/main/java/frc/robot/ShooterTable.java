package frc.robot;

/**
 * Lookup table that maps distance (inches) to shooter velocity (RPS) and hood position (rotations).
 * Uses linear interpolation between data points and clamps to nearest value outside the range.
 */
public class ShooterTable {

    // Limelight mounting constants
    public static final double kLimelightHeightInches = 29.0;   // height of limelight lens off ground
    public static final double kTargetHeightInches    = 65;   // height of AprilTag center off ground 44.5
    public static final double kLimelightAngleDegrees = 22.554012;   // limelight tilt angle from horizontal

    /**
     * Lookup table: { distanceInches, flywheelRPS, hoodPosition }
     */
    private static final double[][] TABLE = {
        // dist (in)  flywheel (RPS)  hood (rot)
        {  10.0,       30.0,           -0.5  },
        {  20.0,       30.0,           -2.0  },
        {  30.0,       35.0,           -2.5  },
        {  40.0,       40.0,           -3.0  },
        {  50.0,       50.0,           -3.5  },
        {  60.0,       50.0,           -4.0  },
        {  70.0,       60.0,           -4.5  },
        {  80.0,       55.0,           -5.0  },
        {  90.0,       78.0,           -5.5  },
        { 100.0,       78.0,           -6.0  },
        { 110.0,       78.0,           -6.0  },
        { 120.0,       78.0,           -6.0  },
        { 130.0,       78.0,           -6.0  },
    };

    /**
     * Calculates distance to the AprilTag in inches using the Limelight's ty value.
     * Formula: distance = (targetHeight - limelightHeight) / tan(limelightAngle + ty)
     */
    public static double getDistanceInches(double ty) {
        double angleRadians = Math.toRadians(kLimelightAngleDegrees + ty);
        return (kTargetHeightInches - kLimelightHeightInches) / Math.tan(angleRadians);
    }

    /** Returns the interpolated flywheel velocity (RPS) for a given distance. */
    public static double getFlywheelRPS(double distanceInches) {
        return interpolate(distanceInches, 1);
    }

    /** Returns the interpolated hood position (rotations) for a given distance. */
    public static double getHoodPosition(double distanceInches) {
        return interpolate(distanceInches, 2);
    }

    /**
     * Linearly interpolates (or clamps) a value from the table for a given distance.
     * distanceInches distance to target
     * col column index: 1 = flywheel, 2 = hood
     */
    private static double interpolate(double distanceInches, int col) {
        // Clamp below minimum
        if (distanceInches <= TABLE[0][0]) {
            return TABLE[0][col];
        }
        // Clamp above maximum
        if (distanceInches >= TABLE[TABLE.length - 1][0]) {
            return TABLE[TABLE.length - 1][col];
        }
        // Find surrounding points and interpolate
        for (int i = 0; i < TABLE.length - 1; i++) {
            if (distanceInches >= TABLE[i][0] && distanceInches <= TABLE[i + 1][0]) {
                double t = (distanceInches - TABLE[i][0]) / (TABLE[i + 1][0] - TABLE[i][0]);
                return TABLE[i][col] + t * (TABLE[i + 1][col] - TABLE[i][col]);
            }
        }
        return TABLE[TABLE.length - 1][col];
    }
}
