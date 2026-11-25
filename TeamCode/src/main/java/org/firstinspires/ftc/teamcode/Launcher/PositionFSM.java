
package org.firstinspires.ftc.teamcode.Launcher;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.util.InterpLUT;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Core.Logger;
import org.firstinspires.ftc.teamcode.Core.Pinpoint;
import org.firstinspires.ftc.teamcode.Core.RobotSettings;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@Config
public class PositionFSM {
    public enum State {
        IN_RANGE,
        OUT_OF_RANGE
    }

    public static double TURRET_X = 0;
    public static double TURRET_Y = 0;
    public static double TURRET_TRIM = 0;
    public static double OUT_OF_RANGE_TOLERANCE = 5;

    public static int RECOVERY_READS = 2;
    public static double MAX_ROBOT_SPEED = 100;
    public static double MAX_TURN_SPEED = 720;
    public static double POSITION_SLACK = 3;
    public static double HEADING_SLACK = 12;

    public static final double[] DISTANCES = {25, 31, 38, 45, 50, 60};
    public static final double[] RPMS = {1400, 1450, 1475, 1475, 1500, 1500};
    public static final double[] HOOD_POSITIONS = {0.000, 0.100, 0.300, 0.300, 0.333, 0.367};

    private final Pinpoint pinpoint;
    private final Logger logger;
    private final RobotSettings.Alliance alliance;
    private RobotSettings.Cell cell;
    private InterpLUT rpmMap, hoodMap;
    private State state = State.IN_RANGE;

    private double goalX = 0, goalY = 0;
    private double distance = 0;
    private double robotBearing = 0;
    private double turretTarget = 0;
    private double targetRPM = 0;
    private double hoodTarget = 0;
    private boolean usingBackEnd = true;
    private boolean firstUpdate = true;

    private boolean trackingValid = false;
    private boolean hasTrustedPose = false;
    private boolean recovering = false;
    private int recoveryGoodReads = 0;
    private double trustedX, trustedY, trustedHeading;
    private double recoveryX, recoveryY, recoveryHeading;
    private final ElapsedTime sinceTrustedPose = new ElapsedTime();
    private final ElapsedTime sinceRecoveryRead = new ElapsedTime();
    private String trackingStatus = "WAITING FOR START POSE";

    public PositionFSM(Pinpoint pinpoint, RobotSettings settings, Logger logger) {
        this.pinpoint = pinpoint;
        this.logger = logger;
        alliance = settings.alliance;
        cell = alliance.startUpCell();
        buildMaps();
    }

    public void buildMaps() {
        if (DISTANCES.length < 2 || DISTANCES.length != RPMS.length || DISTANCES.length != HOOD_POSITIONS.length) {
            throw new IllegalArgumentException("PositionFSM tables need at least 2 points and the same number of distances, rpms and hood positions");
        }
        for (int i = 1; i < DISTANCES.length; i++) {
            if (DISTANCES[i] <= DISTANCES[i - 1]) {
                throw new IllegalArgumentException("PositionFSM DISTANCES have to go up, check point " + i);
            }
        }

        rpmMap = new InterpLUT();
        hoodMap = new InterpLUT();
        for (int i = 0; i < DISTANCES.length; i++) {
            rpmMap.add(DISTANCES[i], RPMS[i]);
            hoodMap.add(DISTANCES[i], HOOD_POSITIONS[i]);
        }
        rpmMap.createLUT();
        hoodMap.createLUT();
    }

    public void updateState() {
        trackingValid = false;

        try {
            pinpoint.update();
        } catch (RuntimeException e) {
            rejectPose("PINPOINT READ FAILED");
            return;
        }

        if (!pinpoint.positionSet()) {
            trackingStatus = "WAITING FOR START POSE";
            return;
        }

        if (!"READY".equals(pinpoint.getStatus())) {
            rejectPose("PINPOINT " + pinpoint.getStatus());
            return;
        }

        double x = pinpoint.getX();
        double y = pinpoint.getY();
        double headingDeg = pinpoint.getHeading();

        if (!Double.isFinite(x) || !Double.isFinite(y)
                || !Double.isFinite(headingDeg)
                || Math.abs(x) > 72 || Math.abs(y) > 72) {
            rejectPose("INVALID POSE");
            return;
        }

        headingDeg = wrap180(headingDeg);

        if (hasTrustedPose && !plausible(x, y, headingDeg,
                trustedX, trustedY, trustedHeading, sinceTrustedPose.seconds())) {
            rejectPose("POSE JUMP REJECTED");
            return;
        }

        if (recovering) {
            if (recoveryGoodReads > 0 && !plausible(x, y, headingDeg,
                    recoveryX, recoveryY, recoveryHeading, sinceRecoveryRead.seconds())) {
                rejectPose("INCONSISTENT RECOVERY POSE");
                return;
            }

            recoveryX = x;
            recoveryY = y;
            recoveryHeading = headingDeg;
            recoveryGoodReads++;
            sinceRecoveryRead.reset();

            if (recoveryGoodReads < Math.max(1, RECOVERY_READS)) {
                trackingStatus = "VERIFYING RECOVERY";
                return;
            }
        }

        trustedX = x;
        trustedY = y;
        trustedHeading = headingDeg;
        hasTrustedPose = true;
        recovering = false;
        recoveryGoodReads = 0;
        sinceTrustedPose.reset();

        double heading = Math.toRadians(headingDeg);
        double turretX = x + TURRET_X * Math.cos(heading) - TURRET_Y * Math.sin(heading);
        double turretY = y + TURRET_X * Math.sin(heading) + TURRET_Y * Math.cos(heading);

        Pose2D goal = alliance.getCell(cell);
        goalX = goal.getX(DistanceUnit.INCH);
        goalY = goal.getY(DistanceUnit.INCH);
        double dx = goalX - turretX;
        double dy = goalY - turretY;
        distance = Math.hypot(dx, dy);

        double fieldBearing = Math.toDegrees(Math.atan2(dy, dx));
        robotBearing = wrap180(fieldBearing - headingDeg);

        double t = robotBearing - 180 + TURRET_TRIM;
        while (t > 0) t -= 360;
        while (t <= -360) t += 360;

        double lo = TurretFSM.minTarget(), hi = TurretFSM.maxTarget();

        if (t >= lo && t <= hi) {
            turretTarget = t;
            state = State.IN_RANGE;
            usingBackEnd = (hi - t) < (t - lo);
        } else {
            double toBack = Math.abs(wrap180(t - hi));
            double toRight = Math.abs(wrap180(t - lo));
            if (firstUpdate) usingBackEnd = toBack <= toRight;
            turretTarget = usingBackEnd ? hi : lo;
            double miss = usingBackEnd ? toBack : toRight;
            state = miss > OUT_OF_RANGE_TOLERANCE ? State.OUT_OF_RANGE : State.IN_RANGE;
        }

        firstUpdate = false;

        double first = DISTANCES[0], last = DISTANCES[DISTANCES.length - 1];
        double d = Range.clip(distance, first + 0.001, last - 0.001);
        targetRPM = rpmMap.get(d);
        hoodTarget = hoodMap.get(d);

        trackingValid = true;
        trackingStatus = state == State.OUT_OF_RANGE ? "OUT OF RANGE - TURN ROBOT" : "TRACKING";
    }

    private void rejectPose(String reason) {
        recovering = true;
        recoveryGoodReads = 0;
        trackingStatus = reason + " - WAITING FOR GOOD POSE";
    }

    private boolean plausible(double x, double y, double heading,
                              double oldX, double oldY, double oldHeading, double dt) {
        dt = Math.max(0, dt);
        double maxDistance = POSITION_SLACK + MAX_ROBOT_SPEED * dt;
        double maxTurn = HEADING_SLACK + MAX_TURN_SPEED * dt;

        return Math.hypot(x - oldX, y - oldY) <= maxDistance
                && Math.abs(wrap180(heading - oldHeading)) <= maxTurn;
    }

    private static double wrap180(double angle) {
        angle %= 360.0;
        if (angle > 180) angle -= 360;
        if (angle <= -180) angle += 360;
        return angle;
    }

    public boolean isTrackingValid() {
        return trackingValid;
    }

    public String getTrackingStatus() {
        return trackingStatus;
    }

    public void switchCell() {
        cell = cell.other();
    }

    public void setCell(RobotSettings.Cell cell) {
        this.cell = cell;
    }

    public RobotSettings.Cell getCell() {
        return cell;
    }

    public boolean OUT_OF_RANGE() {
        return !trackingValid || state == State.OUT_OF_RANGE;
    }

    public double getDistance() {
        return distance;
    }

    public double getTurretTarget() {
        return turretTarget;
    }

    public double getTargetRPM() {
        return targetRPM;
    }

    public double getHoodTarget() {
        return hoodTarget;
    }

    public void log() {
        logger.log("<font color='yellow'>----------POSITION----------</font>", "", Logger.LogLevels.PRODUCTION);
        logger.log("Position state", !trackingValid ? "<font color='red'>" + trackingStatus + "</font>" : state == State.OUT_OF_RANGE ? "<font color='orange'>OUT OF RANGE, turn the robot</font>" : "in range", Logger.LogLevels.PRODUCTION);
        logger.log("Aiming at", alliance + " " + cell + " cell", Logger.LogLevels.PRODUCTION);

        if (pinpoint.getJumpCount() > 0) {
            logger.log("<b><font color='red'>PINPOINT JUMPED</font></b>", pinpoint.getJumpCount() + "x, last " + pinpoint.getLastJump() + (trackingValid ? ", tracking valid" : ", " + trackingStatus), Logger.LogLevels.PRODUCTION);
        }

        logger.log("Goal distance", String.format("%.1f in", distance), Logger.LogLevels.PRODUCTION);
        logger.log("<b><font color='purple'>Pinpoint X</font></b>", String.format("%.1f", pinpoint.getX()), Logger.LogLevels.DEBUG);
        logger.log("<b><font color='purple'>Pinpoint Y</font></b>", String.format("%.1f", pinpoint.getY()), Logger.LogLevels.DEBUG);
        logger.log("<b><font color='purple'>Pinpoint Heading</font></b>", String.format("%.1f", pinpoint.getHeading()), Logger.LogLevels.DEBUG);
        logger.log("<b><font color='purple'>Pinpoint status</font></b>", pinpoint.getStatus(), Logger.LogLevels.DEBUG);
        logger.log("<b><font color='purple'>Start pose sent</font></b>", pinpoint.getTimesSent() + "x", Logger.LogLevels.DEBUG);
        logger.log("Goal x / y", String.format("%.1f / %.1f", goalX, goalY), Logger.LogLevels.DEBUG);
        logger.log("Goal bearing (robot)", String.format("%.1f", robotBearing), Logger.LogLevels.DEBUG);
        logger.log("Turret target from position", String.format("%.1f", turretTarget), Logger.LogLevels.DEBUG);
        logger.log("Table RPM / hood", String.format("%.0f / %.3f", targetRPM, hoodTarget), Logger.LogLevels.DEBUG);
    }
}
