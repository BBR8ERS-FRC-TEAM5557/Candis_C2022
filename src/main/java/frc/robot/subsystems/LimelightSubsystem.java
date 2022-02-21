package frc.robot.subsystems;

import edu.wpi.first.networktables.*;

public class LimelightSubsystem {

    private static LimelightSubsystem instance = null;

    public static LimelightSubsystem getInstance() {
        if (instance == null) {
            instance = new LimelightSubsystem();
        }
        return instance;
    }

    private NetworkTable table;

    public LimelightSubsystem() {
        table = NetworkTableInstance.getDefault().getTable("limelight");
    }

    public double getAngleX() {
        return table.getEntry("tx").getDouble(0);
    }

    public double getAngleY() {
        return table.getEntry("ty").getDouble(0);
    }

    public double getArea() {
        return table.getEntry("ta").getDouble(0);
    }

    public double getSkew() {
        return table.getEntry("ts").getDouble(0);
    }

    public boolean hasTarget() {
        return table.getEntry("tv").getDouble(0) == 0 ? false : true;
    }

    public void enableLEDs() {
        table.getEntry("ledMode").setDouble(3);
    }

    public void disableLEDs() {
        table.getEntry("ledMode").setDouble(1);
    }

    public void enableDriverMode() {
        table.getEntry("camMode").setDouble(1);
    }

    public void disableDriverMode() {
        table.getEntry("camMode").setDouble(0);
    }
}