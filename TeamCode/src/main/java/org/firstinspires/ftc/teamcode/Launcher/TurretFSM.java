package org.firstinspires.ftc.teamcode.Launcher;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Core.HWMap;
import org.firstinspires.ftc.teamcode.Core.Logger;
import org.firstinspires.ftc.teamcode.Core.MotorWrapper;

@Config
public class TurretFSM {
    public enum State {
        ALIGNING,
        ALIGNED,
        FAULTED
    }

    public static double gearRatio = 13.0 / 65.0;
    public static boolean INVERT_MOTOR = false;

    public static double P = 0.015, I = 0, D = 0.0007, F = 0.15;
    public static double TOLERANCE = 2;
    public static double STOPPED_SPEED = 20;
    public static double POWER_CAP = 0.4;

    public static double MAX_ANGLE = 0;
    public static double MIN_ANGLE = -265;
    public static double TARGET_MARGIN = 4;
    public static double FAULT_MARGIN = 7;
    public static double SLOW_ZONE = 55;
    public static double MIN_SLOW_POWER = 0.1;
    public static double LOOKAHEAD_S = 0.1;

    public static double CHECK_POWER = 0.3;
    public static int STUCK_LOOPS = 3;

    public static double POWER_SLEW = 4.0;
    private static final double MAX_SLEW_DT = 0.05;

    private final MotorWrapper turretMotor;
    private final Logger logger;
    private final PIDController pid;
    private final ElapsedTime loopTimer = new ElapsedTime();

    private State state = State.ALIGNING;
    private double target = 0;
    private double currentPos = 0, velocity = 0, predictedPos = 0;
    private double lastPower = 0, lastTicks = 0;
    private int stuckCount = 0;
    private String faultReason = "none";
    private String limitedBy = "none";

    public TurretFSM(HWMap hwMap, Logger logger) {
        this.logger = logger;
        hwMap.getTurretMotor().setInverted(INVERT_MOTOR);
        hwMap.getTurretMotor().setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        turretMotor = new MotorWrapper(hwMap.getTurretMotor(), false, gearRatio, false);
        turretMotor.resetEncoder();
        turretMotor.set(0);
        lastTicks = turretMotor.readandGetPosTicks();
        pid = new PIDController(P, I, D);
        loopTimer.reset();
    }

    public static double minTarget() {
        return MIN_ANGLE + TARGET_MARGIN;
    }

    public static double maxTarget() {
        return MAX_ANGLE - TARGET_MARGIN;
    }

    public void setTargetAngle(double angle) {
        target = Range.clip(angle, minTarget(), maxTarget());
    }

    public void updateState() {
        updateState(true);
    }

    public void updateState(boolean trackingValid) {
        double dt = loopTimer.seconds();
        loopTimer.reset();

        turretMotor.readPosition();
        turretMotor.readVelocity();
        currentPos = turretMotor.getScaledPos();
        velocity = turretMotor.getVelocity() / turretMotor.getTicksPerRev() * 360.0 * gearRatio;
        predictedPos = currentPos + velocity * LOOKAHEAD_S;

        checkFaults();

        if (!trackingValid) {
            turretMotor.set(0);
            lastPower = 0;
            stuckCount = 0;
            pid.reset();
            limitedBy = "tracking lost";
            if (state != State.FAULTED) state = State.ALIGNING;
            return;
        }

        double error = target - currentPos;
        boolean stopped = Math.abs(velocity) < STOPPED_SPEED;

        pid.setPID(P, I, D);
        pid.setTolerance(TOLERANCE);
        double requested = pid.calculate(currentPos, target);
        if (Math.abs(error) < TOLERANCE && stopped) {
            requested = 0;
        } else if (Math.abs(error) >= TOLERANCE && stopped) {
            requested += F * Math.signum(error);
        }

        double power = applyHardstops(requested, dt);
        turretMotor.set(power);
        lastPower = power;

        if (state == State.FAULTED) {
            return;
        }
        state = (Math.abs(error) < TOLERANCE && stopped) ? State.ALIGNED : State.ALIGNING;
    }

    private void checkFaults() {
        if (currentPos > MAX_ANGLE + FAULT_MARGIN || currentPos < MIN_ANGLE - FAULT_MARGIN) {
            fault(String.format("past limit at %.1f", currentPos));
        }

        double ticks = turretMotor.readandGetPosTicks();
        if (Math.abs(lastPower) >= CHECK_POWER && ticks == lastTicks) {
            stuckCount++;
            if (stuckCount >= STUCK_LOOPS) {
                fault(String.format("encoder not moving at power %.2f", lastPower));
            }
        } else {
            stuckCount = 0;
        }
        lastTicks = ticks;
    }

    private double applyHardstops(double requested, double dt) {
        double power = Range.clip(requested, -POWER_CAP, POWER_CAP);
        limitedBy = "none";

        double step = POWER_SLEW * Math.min(dt, MAX_SLEW_DT);
        double maxMagnitude = (Math.signum(power) == Math.signum(lastPower))
                ? Math.abs(lastPower) + step
                : step;

        if (Math.abs(power) > maxMagnitude) {
            power = Math.signum(power) * maxMagnitude;
            limitedBy = "slew";
        }

        double cap = POWER_CAP;
        if (power > 0) cap = zoneCap(MAX_ANGLE - predictedPos);
        if (power < 0) cap = zoneCap(predictedPos - MIN_ANGLE);

        if (Math.abs(power) > cap) {
            power = Math.signum(power) * cap;
            limitedBy = "slow zone";
        }

        if (power > 0 && (predictedPos >= MAX_ANGLE || currentPos >= MAX_ANGLE)) {
            power = 0;
            limitedBy = "back limit";
        }

        if (power < 0 && (predictedPos <= MIN_ANGLE || currentPos <= MIN_ANGLE)) {
            power = 0;
            limitedBy = "right limit";
        }

        if (state == State.FAULTED) {
            power = 0;
            limitedBy = "fault";
        }

        return power;
    }

    private double zoneCap(double distToLimit) {
        if (distToLimit <= 0) return 0;
        if (distToLimit >= SLOW_ZONE) return POWER_CAP;
        return MIN_SLOW_POWER + (POWER_CAP - MIN_SLOW_POWER) * (distToLimit / SLOW_ZONE);
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
        return currentPos;
    }

    public double getTargetAngle() {
        return target;
    }

    public void log() {
        logger.log("<font color='yellow'>----------TURRET----------</font>", "", Logger.LogLevels.PRODUCTION);
        logger.log("Turret state", state == State.FAULTED ? "<font color='red'>FAULTED - " + faultReason + "</font>" : state, Logger.LogLevels.PRODUCTION);
        logger.log("Turret target angle", String.format("%.1f", target), Logger.LogLevels.PRODUCTION);
        logger.log("Turret current angle", String.format("%.1f", currentPos), Logger.LogLevels.PRODUCTION);
        logger.log("Turret velocity", String.format("%.1f", velocity), Logger.LogLevels.DEBUG);
        logger.log("Turret power", String.format("%.2f", lastPower), Logger.LogLevels.DEBUG);
        logger.log("Turret limited by", limitedBy, Logger.LogLevels.DEBUG);
    }
}