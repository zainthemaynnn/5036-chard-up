package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Level;
import frc.robot.Pickup;

public class Vision implements Subsystem {
    private static final double H_LENS = 0.0;
    private static final double H_MID = 0.0;
    private static final double H_HI = 0.0;

    private static NetworkTableInstance net = NetworkTableInstance.getDefault();
    private NetworkTable feed;
    private NetworkTable info;

    public Vision() {
        feed = net.getTable("limelight");
        info = net.getTable("test");
    }

    public double distFrom(Level level, Pickup pickup) {
        if (level == Level.LOW && pickup == Pickup.CONE) {
            double ty = feed.getEntry("ty").getDouble(90.0);
            return ty < 90.0 ? (H_MID - H_LENS) / Math.tan(ty) : -1;
        } else if (level == Level.MID && pickup == Pickup.CONE) {
            double ty = feed.getEntry("ty").getDouble(90.0);
            return ty < 90.0 ? (H_HI - H_LENS) / Math.tan(ty) : -1;
        } else if (level == Level.LOW && pickup == Pickup.CUBE) {

        } else if (level == Level.MID && pickup == Pickup.CUBE) {
            
        } else {
            return -1;
        }
    }

    double n = 0.0;
    @Override
    public void periodic() {
        System.out.println(midDist());
        info.getEntry("x").setDouble(n++);
    }
}
