package frc.robot;

public final class ShooterTable {
    private ShooterTable() {}

    /**
     * Table format:
     * { distanceInches, flywheelRPS, hoodPosition }
     */
    private static final double[][] TABLE = {
        // dist (in), flywheel (RPS), hood (rot) 
         { 30.0, 31.0, 0 },
         { 40.0, 31.0, 0 },
         { 50.0, 31.0, -7 },
         { 60.0, 28.0, -7.5 },
         { 70.0, 29.0, -8.5 },
         { 80.0, 31.0, -9 },
         { 90.0, 31, -9 },
         { 100.0, 31.0, -9 },
         { 110.0, 31.0, -9 }, 
         { 120.0, 32.0, -10 },
         { 130.0, 33.0, -12 },
         { 140.0, 35.0, -12 },
         { 150.0, 37.0, -10 },
         { 160.0, 38.0, -10 },
         { 170.0, 39.0, -10 },
         { 180.0, 39.5, -10 },
         //Old havent done yet
         { 210.0, 41.0, -11 },
         { 240.0, 43.0, -11 },
         { 250.0, 45.0, -11 },
         { 290.0, 45.0, -12.5 },

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
