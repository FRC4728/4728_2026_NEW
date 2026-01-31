package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class LimelightIO {
    private final NetworkTableEntry tv;
    private final NetworkTableEntry tx;

    public LimelightIO(String name){
        NetworkTable table = NetworkTableInstance.getDefault().getTable(name);
        tv = table.getEntry("tv");
        tx = table.getEntry("tx");
    }

    public boolean hasTarget(){
        return tv.getDouble(0.0) == 1.0;
    }

    public double txDeg(){
        return tx.getDouble(0.0);
    }

}
