package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Level;
import frc.robot.Pickup;

public class Vision implements Subsystem {
    private static final double H_LENS = 11.75;
    private static final double A_LENS = Math.toRadians(70.0);
    private static final double H_CONE_MID = 22.125;
    private static final double H_CONE_HI = 0.0;
    private static final double H_CUBE_MID = 0.0;
    private static final double H_CUBE_HI = 0.0;

    private static NetworkTableInstance net = NetworkTableInstance.getDefault();
    private NetworkTable feed;
    private NetworkTable info;

    private enum Tag {
        EMPTY           (0),
        RED_1           (3),
        RED_2           (2),
        RED_3           (1),
        RED_TERMINAL    (5),
        BLUE_1          (8),
        BLUE_2          (7),
        BLUE_3          (6),
        BLUE_TERMINAL   (4);

        private int id;

        private Tag(int id) {
            this.id = id;
        }

        public int id() {
            return this.id;
        }

        public static Tag fromId(int id) {
            switch (id) {
                case 0:
                    return EMPTY;
                case 1:
                    return RED_3;
                case 2:
                    return RED_2;
                case 3:
                    return RED_1;
                case 4:
                    return BLUE_TERMINAL;
                case 5:
                    return RED_TERMINAL;
                case 6:
                    return BLUE_3;
                case 7:
                    return BLUE_2;
                case 8:
                    return BLUE_1;
                default:
                    throw new IllegalArgumentException("unsupported tag");
            }
        }
    };

    public Vision() {
        feed = net.getTable("limelight");
        Shuffleboard.getTab("Debugging").addDouble("x", () -> distFrom(Level.MID, Pickup.CONE));
        Shuffleboard.getTab("Debugging").addDouble("tx", () -> feed.getEntry("tx").getDouble(90.0));
        Shuffleboard.getTab("Debugging").addDouble("ty", () -> feed.getEntry("ty").getDouble(90.0));
    }

    private double distFromLowerTargetH(double h) {
        double ty = -feed.getEntry("ty").getDouble(90.0);
        return ty < 90.0 ? (h - H_LENS) / Math.tan(Math.toRadians(ty)) : -1;
    }

    public double distFrom(Level level, Pickup pickup) {
        if (level == Level.LOW) {
            return distFromLowerTargetH(pickup == Pickup.CONE ? H_CONE_MID : H_CUBE_MID);
        } else if (level == Level.MID) {
            return distFromLowerTargetH(pickup == Pickup.CONE ? H_CONE_MID : H_CUBE_MID);
        } else {
            return -1;
        }
    }

    public Pose3d arr6ToPose3d(double[] arr) {
        return new Pose3d(
            new Translation3d(arr[0], arr[1], arr[2]),
            new Rotation3d(arr[3], arr[4], arr[5])
        );
    }

    public Tag tag() {
        return Tag.fromId((int) feed.getEntry("tid").getInteger(-1));
    }

    public Pose3d poseTagRelative() {
        return arr6ToPose3d(feed
            .getEntry("camerapose_targetspace")
            .getDoubleArray(new double[] { 0, 0, 0, 0, 0, 0 })
        );
    }
}
