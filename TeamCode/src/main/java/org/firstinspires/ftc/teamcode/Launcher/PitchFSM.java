package org.firstinspires.ftc.teamcode.Launcher;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Core.HWMap;
import org.firstinspires.ftc.teamcode.Core.Logger;

@Config
public class PitchFSM {
    public enum State {
        ALIGNING,
        ALIGNED,
        FAULTED
    }

    public static double gearRatio = 3.0 / 29.0;
    public static boolean INVERT_POWER = false;
    public static boolean INVERT_ENCODER = true;

    public static double MIN_ANGLE = 0;
    public static double MAX_ANGLE = 30;
    public static double FAULT_MARGIN = 3;

    public static double P = 0.03, I = 0, D = 0, F = 0.05;
    public static double TOLERANCE = 0.5;
    public static double POWER_CAP = 0.6;

    public static double CHECK_POWER = 0.3;
    public static double CHECK_TIME = 0.25;
    public static double CHECK_MIN_MOVE = 0.3;

    private final CRServo pitchServo;
    private final Logger logger;
    private final AnalogInput pitchEncoder;
    private final PIDController pid;
    private final ElapsedTime checkTimer = new ElapsedTime();

    private State state = State.ALIGNING;
    private double target = 0;
    private double lastRaw, servoDeg = 0, hoodAngle = 0;
    private double lastPower = 0;
    private boolean checking = false;
    private double checkStartAngle = 0;
    private String faultReason = "none";
    private boolean zeroed = false;

    public PitchFSM(HWMap hwMap, Logger logger) {
        this.logger = logger;
        pitchServo = hwMap.getPitchServo();
        pitchEncoder = hwMap.getPitchEncoder();
        pitchServo.set(0);
        lastRaw = rawAngle();
        pid = new PIDController(P, I, D);
    }

    public void setTargetAngle(double angle) {
        target = Range.clip(angle, MIN_ANGLE, MAX_ANGLE);
    }

    public void updateState() {
        if (!zeroed) {
            lastRaw = rawAngle();
            servoDeg = 0;
            checkTimer.reset();
            zeroed = true;
        }
        readHood();
        checkFaults();

        double error = target - hoodAngle;
        pid.setPID(P, I, D);
        pid.setTolerance(TOLERANCE);
        double power = pid.calculate(hoodAngle, target);
        if (Math.abs(error) > TOLERANCE) {
            power += F * Math.signum(error);
        }
        power = Range.clip(power, -POWER_CAP, POWER_CAP);

        if (power > 0 && hoodAngle >= MAX_ANGLE) power = 0;
        if (power < 0 && hoodAngle <= MIN_ANGLE) power = 0;
        if (state == State.FAULTED) power = 0;

        pitchServo.set(INVERT_POWER ? -power : power);
        lastPower = power;

        if (state == State.FAULTED) {
            return;
        }
        state = Math.abs(error) < TOLERANCE ? State.ALIGNED : State.ALIGNING;
    }

    private double rawAngle() {
        double raw = pitchEncoder.getVoltage() / 3.3 * 360.0;
        return INVERT_ENCODER ? 360.0 - raw : raw;
    }

    private void readHood() {
        double raw = rawAngle();
        double delta = raw - lastRaw;
        if (delta > 180) delta -= 360;
        else if (delta < -180) delta += 360;
        servoDeg += delta;
        lastRaw = raw;
        hoodAngle = servoDeg * gearRatio;
    }

    private void checkFaults() {
        if (hoodAngle < MIN_ANGLE - FAULT_MARGIN) {
            fault(String.format("below the bottom (%.1f), servo and encoder directions disagree", hoodAngle));
        }
        if (hoodAngle > MAX_ANGLE + FAULT_MARGIN) {
            fault(String.format("past the top (%.1f)", hoodAngle));
        }

        if (Math.abs(lastPower) >= CHECK_POWER) {
            if (!checking) {
                checking = true;
                checkStartAngle = hoodAngle;
                checkTimer.reset();
            } else if (checkTimer.seconds() >= CHECK_TIME) {
                if (Math.abs(hoodAngle - checkStartAngle) < CHECK_MIN_MOVE) {
                    fault(String.format("hood not moving at power %.2f", lastPower));
                }
                checkStartAngle = hoodAngle;
                checkTimer.reset();
            }
        } else {
            checking = false;
        }
    }

    private void fault(String reason) {
        if (state != State.FAULTED) {
            state = State.FAULTED;
            faultReason = reason;
        }
    }

    public boolean ALIGNED() {
        return state == State.ALIGNED;
    }

    public boolean FAULTED() {
        return state == State.FAULTED;
    }

    public String getFaultReason() {
        return faultReason;
    }

    public double getCurrentAngle() {
        return hoodAngle;
    }

    public void log() {
        logger.log("<font color='yellow'>----------PITCH----------</font>", "", Logger.LogLevels.PRODUCTION);
        logger.log("Pitch state", state == State.FAULTED ? "<font color='red'>FAULTED - " + faultReason + "</font>" : state, Logger.LogLevels.PRODUCTION);
        logger.log("Pitch target angle", String.format("%.1f", target), Logger.LogLevels.PRODUCTION);
        logger.log("Pitch current angle", String.format("%.1f", hoodAngle), Logger.LogLevels.PRODUCTION);
        logger.log("Pitch power", String.format("%.2f", lastPower), Logger.LogLevels.DEBUG);
        logger.log("Pitch servo deg from bottom", String.format("%.1f", servoDeg), Logger.LogLevels.DEBUG);
    }
}
