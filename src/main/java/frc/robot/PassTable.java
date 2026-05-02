package frc.robot;

/**
 * Lookup table for pass shots, keyed on distance from the robot to the pass target.
 * Format: { distanceInches, flywheelRPS, hoodPosition, airtimeSeconds }
 */
public final class PassTable {
    private PassTable() {}

    private static final double[][] TABLE = {
        // dist (in), flywheel (RPS), hood (rot), airtime (s)
        { 200.0, 60.0, -15.0, 1.2 },
        { 250.0, 70.0, -15.0, 1.3 },
        { 300.0, 80.0, -15.0, 1.3 },
        { 350.0, 90.0, -15.0, 1.4 }, //60
        { 400.0, 100.0, -15.0, 1.4 }, //70
        { 450.0, 100.0, -12.0, 1.5 },
        { 500.0, 100.0, -12.0, 1.5 },
        { 550.0, 100.0, -12.0, 1.6 },
        { 600.0, 100.0, -12.0, 1.6 },
        { 650.0, 100.0, -12.0, 1.7 },
        { 700.0, 100.0, -12.0, 1.7 },
    };

    public static double getFlywheelRPS(double distanceInches) {
        return interpolate(distanceInches, 1);
    }

    public static double getHoodPosition(double distanceInches) {
        return interpolate(distanceInches, 2);
    }

    public static double getAirtime(double distanceInches) {
        return interpolate(distanceInches, 3);
    }

    private static double interpolate(double x, int column) {
        if (TABLE.length == 0) {
            throw new IllegalStateException("PassTable TABLE cannot be empty");
        }

        if (x <= TABLE[0][0]) {
            return TABLE[0][column];
        }

        if (x >= TABLE[TABLE.length - 1][0]) {
            return TABLE[TABLE.length - 1][column];
        }

        for (int i = 0; i < TABLE.length - 1; i++) {
            double x1 = TABLE[i][0];
            double x2 = TABLE[i + 1][0];

            if (x >= x1 && x <= x2) {
                double y1 = TABLE[i][column];
                double y2 = TABLE[i + 1][column];
                double t = (x - x1) / (x2 - x1);
                return y1 + t * (y2 - y1);
            }
        }

        return TABLE[TABLE.length - 1][column];
    }
}