package frc.robot;

/**
 * Lookup table that maps distance (inches) to shooter velocity (RPS) and hood position (rotations).
 * Uses linear interpolation between data points and clamps to nearest value outside the range.
 */
public class ShooterTable {

    // Limelight mounting constants
    public static final double kLimelightHeightInches = 29.0;   // height of limelight lens off ground
    public static final double kTargetHeightInches    = 65;   // height of AprilTag center off ground 44.5 65
    public static final double kLimelightAngleDegrees = 22.54012;//22.554012;   // limelight tilt angle from horizontal

    /**
     * Lookup table: { distanceInches, flywheelRPS, hoodPosition }
     */
    private static final double[][] TABLE = {
        // dist (in), flywheel (RPS), hood (rot) 
         { 40.0, 31.0, 0 },
         { 50.0, 31.0, 0 },
         { 60.0, 31.0, -7 },
         { 70.0, 28.0, -7.5 },
         { 80.0, 29.0, -8.5 },
         { 90.0, 31.0, -9 },
         { 100.0, 31, -9 },
         { 110.0, 31.0, -9 },
         { 120.0, 31.0, -9 }, 
         { 130.0, 32.0, -10 },
         { 140.0, 33.0, -12 },
         { 150.0, 35.0, -12 },
         { 160.0, 38.0, -10 },
         { 170.0, 39.0, -10 },
         { 180.0, 40.0, -10 },
         { 190.0, 40.0, -10 },
         //Old haven't done yet
         { 214.0, 41.0, -11 },
         { 239.0, 43.0, -11 },
         { 250.0, 45.0, -11 },
         { 290.0, 45.0, -12.5 },

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
