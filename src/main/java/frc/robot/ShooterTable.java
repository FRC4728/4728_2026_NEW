package frc.robot;
 
public final class ShooterTable {
    private ShooterTable() {}
 
    /**
     * Table format:
     * { distanceInches, flywheelRPS, hoodPosition, airtimeSeconds }
     */
    private static final double[][] TABLE = {
        // dist (in), flywheel (RPS), hood (rot), Airtime (s) 
         { 50.0, 34, -3, 1 },
         { 60.0, 32, -5, 1.01 },
         { 70.0, 33, -6, 1.02 }, //32
         { 80.0, 34, -7, 1.02 }, //33
         { 90.0, 34, -8, 1.05 }, //33
         { 100.0, 35, -9, 1.08 }, //33
         { 110.0, 36, -9, 1.11 }, //35
         { 120.0, 39, -9, 1.2 }, //37
         { 130.0, 40, -9, 1.23 }, //38
         { 140.0, 42, -10, 1.24 }, //38
         { 150.0, 44, -10, 1.26 }, //39
         { 160.0, 46, -10, 1.29 }, //40
         { 170.0, 47, -10, 1.27 }, //41
         { 180.0, 54, -10, 1.31 }, //41.5, 49
         { 190.0, 57, -11, 1.33 }, //41, 52
         { 200.0, 58, -11, 1.33 }, //42, 53
         { 210.0, 59, -11, 1.35 }, //43, 53
         { 220.0, 60.0, -10, 1.37 }, //46, 54
         { 230.0, 60.0, -10, 1.4 }, //47, 54
         { 240.90, 100.0, -11, 1.4}
 
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