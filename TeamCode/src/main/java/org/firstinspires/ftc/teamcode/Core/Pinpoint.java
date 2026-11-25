package org.firstinspires.ftc.teamcode.Core;

import com.acmerobotics.dashboard.config.Config;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@Config
public class Pinpoint {
    public RobotSettings robotSettings;
    GoBildaPinpointDriver odo;
    Pose2D pos;
    private Pose2D pendingPose = null;
    private boolean poseSent = false;
    private int loopsSinceSent = 0;
    private final ElapsedTime goodFor = new ElapsedTime();
    private boolean readingGood = false;
    private int timesSent = 0;
    private final ElapsedTime sinceInit = new ElapsedTime();

    public static double VERIFY_TIME = 0.5;
    public static double RESET_SETTLE_TIME = 0.5;

    public static double JUMP_DISTANCE = 12;
    public static double JUMP_HEADING = 90;
    private int skipJumpCheck = 0;
    private boolean hasReading = false;
    private int jumpCount = 0;
    private String lastJump = "none";
    private int badReads = 0;


    public double x;
    public double y;
    public double heading;
    public static double Xoffset = -147.088;
    public static double YOffset = 38.9;


/*    Pose convertedPose = new Pose(
            y + 72,
            72 - x,

            heading - Math.PI / 2
    );*/


    public Pinpoint(MechanumHardwareMap mechanumHardwareMap) {
        odo = mechanumHardwareMap.getOdo();
        odo.setOffsets(Xoffset, YOffset, DistanceUnit.MM);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        robotSettings = RobotSettings.load();
        odo.resetPosAndIMU();
        sinceInit.reset();
    }

    public void update() {
        if (pendingPose != null && !poseSent && sinceInit.seconds() > RESET_SETTLE_TIME
                && odo.getDeviceStatus() == GoBildaPinpointDriver.DeviceStatus.READY) {
            odo.setPosition(pendingPose);
            poseSent = true;
            loopsSinceSent = 0;
            readingGood = false;
            timesSent++;
        }
        odo.update();
        if (odo.getDeviceStatus() == GoBildaPinpointDriver.DeviceStatus.FAULT_BAD_READ) {
            badReads++;
            return;
        }
        pos = odo.getPosition();
        double newX = pos.getX(DistanceUnit.INCH);
        double newY = pos.getY(DistanceUnit.INCH);
        double newHeading = pos.getHeading(AngleUnit.DEGREES);

        if (pendingPose != null && poseSent) {
            loopsSinceSent++;
            boolean matches = Math.hypot(newX - pendingPose.getX(DistanceUnit.INCH), newY - pendingPose.getY(DistanceUnit.INCH)) < 1
                    && Math.abs(AngleUnit.normalizeDegrees(newHeading - pendingPose.getHeading(AngleUnit.DEGREES))) < 2
                    && odo.getDeviceStatus() == GoBildaPinpointDriver.DeviceStatus.READY;
            if (matches) {
                if (!readingGood) goodFor.reset();
                readingGood = true;
                if (goodFor.seconds() >= VERIFY_TIME) {
                    pendingPose = null;
                    skipJumpCheck = 1;
                }
            } else {
                readingGood = false;
                if (loopsSinceSent > 3) poseSent = false;
            }
        }

        if (pendingPose == null && skipJumpCheck > 0) {
            skipJumpCheck--;
        } else if (pendingPose == null && hasReading) {
            double moved = Math.hypot(newX - x, newY - y);
            double turned = Math.abs(AngleUnit.normalizeDegrees(newHeading - heading));
            if (moved > JUMP_DISTANCE || turned > JUMP_HEADING) {
                jumpCount++;
                lastJump = String.format("%.1f, %.1f, %.0f -> %.1f, %.1f, %.0f at %.1f s after init, status %s",
                        x, y, heading, newX, newY, newHeading, sinceInit.seconds(), odo.getDeviceStatus());
            }
        }

        x = newX;
        y = newY;
        heading = newHeading;
        hasReading = true;
    }

    public String getStatus() {
        return odo.getDeviceStatus().toString();
    }

    public int getTimesSent() {
        return timesSent;
    }

    public int getBadReads() {
        return badReads;
    }

    public int getJumpCount() {
        return jumpCount;
    }

    public String getLastJump() {
        return lastJump;
    }

    public double getHeading() {
        return heading;
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public Pose2D getPos() {
        return pos;
    }

    public void setPosition(Pose2D pose) {
        pendingPose = pose;
        poseSent = false;
        readingGood = false;
    }

    public boolean positionSet() {
        return pendingPose == null;
    }

    public void resetIMU() {
        odo.setPosition(new Pose2D(DistanceUnit.INCH, 141.5, 0, AngleUnit.DEGREES, 0));
    }

    // NEW COORDINATE SYSTEM

    // public double x;
    // public double y;
    // public double heading;
    // Pose convertedPose;

    // public Pinpoint(HWMap hwMap) {
    //     odo = hwMap.getOdo();
    //     odo.setOffsets(Xoffset, YOffset, DistanceUnit.MM);
    //     odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
    //     odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
    //     robotSettings = RobotSettings.load();
    //
    //     // Initialize position values after odo is set up
    //     pos = odo.getPosition();
    //     x = pos.getX(DistanceUnit.INCH);
    //     y = pos.getY(DistanceUnit.INCH);
    //     heading = pos.getHeading(AngleUnit.RADIANS);
    //     convertedPose = new Pose(
    //             y + 72,
    //             72 - x,
    //             heading - Math.PI / 2
    //     );
    // }

    // public void update() {
    //     odo.update();
    //     pos = odo.getPosition();
    //     x = pos.getX(DistanceUnit.INCH);
    //     y = pos.getY(DistanceUnit.INCH);
    //     heading = pos.getHeading(AngleUnit.RADIANS);
    //     convertedPose = new Pose(
    //             y + 72,
    //             72 - x,
    //             heading - Math.PI / 2
    //     );
    // }
    //

}
