/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;

/**
 * 1. Limelight configuration - https://docs.limelightvision.io/docs/docs-limelight/pipeline-apriltag/apriltags
 *   Your Limelight should be mounted above or below tag height and angled up/down such that the target is centered. 
 *   Your target should look as trapezoidal as possible from your camera's perspective. 
 *   You don't want your camera to ever be completely "head-on" with a tag if you want to avoid tag flipping.
 * 
 *   a) distance --- https://docs.limelightvision.io/docs/docs-limelight/tutorials/tutorial-estimating-distance
 * 
 *   b) moving in range --- https://docs.limelightvision.io/docs/docs-limelight/tutorials/tutorial-getting-in-range
 * 
 *   c) **doing both --- https://docs.limelightvision.io/docs/docs-limelight/tutorials/tutorial-aiming-and-ranging
 *          use this link. the example code is better. 
 *          https://github.com/LimelightVision/limelight-examples/blob/main/java-wpilib/swerve-aim-and-range/src/main/java/frc/robot/Robot.java
 */
public class Limelight {
    public enum LightMode {
        DEFAULT(0), OFF(1), BLINK(2), ON(3);

        private final int ledMode;

        private LightMode(int ledMode) {
            this.ledMode = ledMode;
        }

        public int getLedMode() {
            return ledMode;
        }
    }

    public enum StreamMode {
        STANDARD(0), MAIN(1), SECONDARY(2);

        private final int mode;

        private StreamMode(int mode) {
            this.mode = mode;
        }

        public int getMode() {
            return mode;
        }
    }

    private static Limelight instance;

    private NetworkTable table;

    //Frequently used entries to store
    private NetworkTableEntry tx;
    private NetworkTableEntry ty;
    private NetworkTableEntry ta;


    private Limelight() {
        table = NetworkTableInstance.getDefault().getTable("limelight");

        tx = table.getEntry("tx");
        ty = table.getEntry("ty");
        ta = table.getEntry("ta");
    }

    //Whether camera has any valid targets
    public boolean hasValidTarget() {
        return (table.getEntry("tv").getDouble(0) == 0) ? false : true;
    }
    
    //Horizontal offset from crosshair to target
    //-27, 27 degrees
    public double getXAngle() {
        return tx.getDouble(0);
    }

    //Vertical offset from crosshair to target
    //-20.5, 20.5 degrees 
    public double getYAngle() {
        return ty.getDouble(0);
    }

    public double getArea() {
        return ta.getDouble(0);
    }

    //-90 to 0 degrees. Rotation of the object
    public double getSkew() {
        return table.getEntry("ts").getDouble(0);
    }

    //Latency in ms of the pipeline
    public double getDeltaTime() {
        return table.getEntry("tl").getDouble(0);
    }

    //The length of the shortest side of the bounding box in pixels
    public double getShortLength() {
        return table.getEntry("tshort").getDouble(0);
    }

    //The length of the longest side of the bounding box in pixels
    public double getLongLength() {
        return table.getEntry("tlong").getDouble(0);
    }

    //The length of the horizontal side of the box (0-320 pixels)
    public double getHorizontalLength() {
        return table.getEntry("thor").getDouble(0);
    }

    //The length of the vertical side of the box (0-320 pixels)
    public double getVerticalLength() {
        return table.getEntry("tvert").getDouble(0);
    }

    //Returns the index of the current vision pipeline (0... 9)
    public int getPipeIndex() {
        return (int) table.getEntry("getpipe").getDouble(0);
    }

    public double[] getXCorners() {
        return table.getEntry("tcornx").getDoubleArray(new double[] {0, 0, 0, 0});
    }

    public double[] getYCorners() {
        return table.getEntry("tcorny").getDoubleArray(new double[] {0, 0, 0, 0});
    }

    //Returns the crosshair position in screen space from -1 to 1
    public double getRawXCrosshair() {
        return table.getEntry("cx0").getDouble(0);
    }

    //Returns the crosshair position in screen space from -1 to 1
    public double getRawYCrosshair() {
        return table.getEntry("cx1").getDouble(0);
    }
    
    //returns the crosshair position in angle space from -27 to 27
    public double getXCrosshair() {
        return getRawXCrosshair() * 27;
    }

    //returns the crosshair position in angle space from -20.5 to 20.5
    public double getYCrosshair() {
        return getRawYCrosshair() * 20.5d;
    }

    public double getDistance() {
        if (!Limelight.getInstance().hasValidTarget()){
           return(-1);
        }
        double h2 = 49.5;
        double h1 = 19;
        double a2 = Math.toRadians(27);
        double a1 = Math.toRadians(getYAngle() + getYCrosshair());
        return ((h2-h1) / Math.tan(a1+a2));
    }

    //Sets the LEDs to either On, Off, Blinking, or determined by the pipeline
    public void setLightState(LightMode lMode) {
        table.getEntry("ledMode").setNumber(lMode.getLedMode());
    }

    public LightMode getLightState() {
        return LightMode.values()[table.getEntry("ledMode").getNumber(0).intValue()];
    }

    //True for human use, false for vision pipeline. Starts false
    //Enabling this makes the exposure something you can easily see out of and disables vision processing
    public void setDriveMode(boolean b) {
        if(b) {
            table.getEntry("camMode").setNumber(1);
        }
        else {
            table.getEntry("camMode").setNumber(0);
        }
    } 

    //Sets the limelights current pipeline
    public void setPipeline(int pipeline) {
        table.getEntry("pipeline").setNumber(pipeline);
    }

    //Sets the layout of the cameras viewed at 10.40.85.11:5800
    //Standard is side by side, Main is Limelight big with secondary camera in bottom right, Secondary is vice versa
    public void setStreamMode(StreamMode streamMode) {
        table.getEntry("stream").setNumber(streamMode.getMode());
    }

    //Enables and disables the camera taking snapshots
    //Camera takes 1 snapshot then sets this value to 0
    public void takeSnapshot() {
            table.getEntry("snapshot").setNumber(1);
    }

    public static Limelight getInstance() {
        if(instance == null) {
            instance = new Limelight();
        }

        return instance;
    }
    
}
