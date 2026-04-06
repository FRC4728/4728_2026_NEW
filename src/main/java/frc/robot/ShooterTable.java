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
         { 90.0, 30, -8, 1.02 },
         { 100.0, 30, -9, 1.08 },
         { 110.0, 32, -9, 1.11 }, 
         { 120.0, 34, -9, 1.2 },
         { 130.0, 35, -9, 1.23 },
         { 140.0, 35, -10, 1.24 },
         { 150.0, 36, -10, 1.26 },
         { 160.0, 37, -10, 1.29 }, // Flywheel 41 Airtime 1.23
         { 170.0, 38, -10, 1.27 }, //Flywheel 42 Airtime 1.21
         { 180.0, 38.5, -11, 1.31 }, //Flywheel 43 AirTime 1.25
         { 190.0, 38, -12, 1.33 }, //Flywheel 45 Airtime 1.27
         { 200.0, 39, -12, 1.33 },
         { 210.0, 40, -12, 1.35 },
         { 220.0, 44.0, -11, 1.37 },
 
    };
 
    public static double getFlywheelRPS(double distanceInches) {
        return interpolate(distanceInches, 1);
    }
 
    public static double getHoodPosition(double distanceInches) {
        return interpolate(distanceInches, 2);
    }
 
    /** Returns the ball airtime in seconds for the given distance. Used for shoot-on-the-move compensation. */
    public static double getAirtime(double distanceInches) {
        return interpolate(distanceInches, 3);
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