package frc.robot;

public final class ShooterTable {
    private ShooterTable() {}

    /**
     * Table format:
     * { distanceInches, flywheelRPS, hoodPosition, airtimeSeconds }
     */
    private static final double[][] TABLE = {
        // dist (in), flywheel (RPS), hood (rot), Airtime (s) 
         { 50.0, 33, -3, 1 },
         { 60.0, 29, -5, 1.01 },
         { 70.0, 29, -6, 1.02 },
         { 80.0, 30, -7, 1.02 },
         { 90.0, 32, -7, 1.02 },
         { 100.0, 33, -7, 1.08 },
         { 110.0, 36, -7.5, 1.11 }, 
         { 120.0, 38, -7.5, 1.2 },
         { 130.0, 39, -8, 1.23 },
         { 140.0, 40, -8, 1,24 },
         { 150.0, 41, -9, 1.26 },
         { 160.0, 41, -9, 1.23 },
         { 170.0, 42, -9.5, 1.21 },
         { 180.0, 43, -9.5, 1.25 },
         { 190.0, 45, -10, 1.27 },
         { 200.0, 42.5, -11, 1.33 },
         { 210.0, 44, -11, 1.35 },
         { 220.0, 44.0, -11, 1.37 },

    };

    public static double getFlywheelRPS(double distanceInches) {
        return interpolate(distanceInches, 1);
    }

    public static double getHoodPosition(double distanceInches) {
        return interpolate(distanceInches, 2);
    }

    private static double interpolate(double x, int column) {
        if (TABLE.length == 0) {
            throw new IllegalStateException("ShooterTable TABLE cannot be empty");
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
