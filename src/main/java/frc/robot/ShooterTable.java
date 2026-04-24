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
         { 70.0, 32, -6, 1.02 }, //29
         { 80.0, 33, -7, 1.02 }, //30
         { 90.0, 33, -8, 1.05 }, //30
         { 100.0, 33, -9, 1.08 }, //30
         { 110.0, 35, -9, 1.11 }, //32
         { 120.0, 37, -9, 1.2 }, //34
         { 130.0, 38, -9, 1.23 }, //35
         { 140.0, 38, -10, 1.24 }, //35
         { 150.0, 39, -10, 1.26 }, //36
         { 160.0, 40, -10, 1.29 }, //37
         { 170.0, 41, -10, 1.27 }, //38
         { 180.0, 41.5, -11, 1.31 }, //38.5
         { 190.0, 41, -12, 1.33 }, //38
         { 200.0, 42, -12, 1.33 }, //39
         { 210.0, 43, -12, 1.35 }, //40
         { 220.0, 46.0, -11, 1.37 }, //44
         { 230.0, 47.0, -11, 1.4 } //45
 
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