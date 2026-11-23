package org.firstinspires.ftc.teamcode.Launcher.testClasses;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Core.HWMap;
import org.firstinspires.ftc.teamcode.Core.MotorWrapper;

/*
 * Turret test:
 *   park the turret facing back, then press start, the motor drives it right away
 *   Cross turns the motor off/on if you need to stop it
 *   dpad right/left moves the target 5 deg right/left
 *   Triangle goes to -90, Circle goes to the right limit, Square goes back to the start
 *   if it turns the wrong way it will fault, restart and press Square during init
 *   R1/L1 change P, raise it until it gets to the target fast
 *   R3/L3 (press the sticks) change D, raise it if it overshoots or rocks back and forth
 *   R2/L2 change F, raise it if it stops short and doesnt move for small steps
 *   dpad up/down change the power cap, start low and raise it once it behaves
 *   it should never go past -4 or -261, and it should slow down near the ends
 *   Status shows what the hardstops are doing, Limited by shows what cut the power
 */
@TeleOp(group = "Launcher", name = "Turret Full Test")
public class OldTurretTest extends LinearOpMode {

    public static double gearRatio = 13.0 / 65.0;

    public static double P = 0.015, I = 0, D = 0.0007, F = 0.15;
    public static double TOLERANCE = 3;
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

    private MotorWrapper turretMotor;
    private PIDController pid;
    private final ElapsedTime loopTimer = new ElapsedTime();

    private boolean invertMotor = false, motorOn = true;
    private double target = 0;
    private double currentPos = 0, velocity = 0, predictedPos = 0;
    private double lastPower = 0, lastTicks = 0;
    private int stuckCount = 0;
    private boolean faulted = false;
    private String faultReason = "none";
    private String limitedBy = "none";

    private boolean lastA, lastB, lastX, lastY, lastUp, lastDown, lastLeft, lastRight, lastRB, lastLB, lastRT, lastLT, lastR3, lastL3;

    @Override
    public void runOpMode() throws InterruptedException {
        HWMap hwMap = new HWMap(hardwareMap);

        while (!isStarted() && !isStopRequested()) {
            if (gamepad1.x && !lastX) invertMotor = !invertMotor;
            lastX = gamepad1.x;
            telemetry.addLine("Park turret facing back before pressing start");
            telemetry.addData("Flip motor (Square)", invertMotor);
            telemetry.update();
        }
        if (isStopRequested()) return;

        hwMap.getTurretMotor().setInverted(invertMotor);
        hwMap.getTurretMotor().setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        turretMotor = new MotorWrapper(hwMap.getTurretMotor(), false, gearRatio, false);
        turretMotor.resetEncoder();
        turretMotor.set(0);
        lastTicks = turretMotor.readandGetPosTicks();
        pid = new PIDController(P, I, D);
        target = 0;
        motorOn = true;
        loopTimer.reset();

        while (opModeIsActive()) {
            double dt = loopTimer.seconds();
            loopTimer.reset();

            readTurret();
            readButtons();
            checkFaults();

            target = Range.clip(target, MIN_ANGLE + TARGET_MARGIN, MAX_ANGLE - TARGET_MARGIN);
            double error = target - currentPos;
            double requested = 0;
            boolean stopped = Math.abs(velocity) < STOPPED_SPEED;
            if (motorOn) {
                pid.setPID(P, I, D);
                pid.setTolerance(TOLERANCE);
                requested = pid.calculate(currentPos, target);
                if (Math.abs(error) < TOLERANCE && stopped) {
                    requested = 0;
                } else if (Math.abs(error) >= TOLERANCE && stopped) {
                    requested += F * Math.signum(error);
                }
            }
            double power = applyHardstops(requested, dt);

            turretMotor.set(power);
            lastPower = power;

            telemetry.addData("Faulted", faulted ? "YES - " + faultReason : "no");
            telemetry.addData("Motor (Cross)", motorOn ? "ON" : "off");
            telemetry.addData("Status", status());
            telemetry.addData("Target (dpad L/R)", target);
            telemetry.addData("Angle", currentPos);
            telemetry.addData("To back limit", MAX_ANGLE - currentPos);
            telemetry.addData("To right limit", currentPos - MIN_ANGLE);
            telemetry.addData("Error", error);
            telemetry.addData("At target", Math.abs(error) < TOLERANCE ? "yes" : "no");
            telemetry.addData("Power", power);
            telemetry.addData("Limited by", limitedBy);
            telemetry.addData("P (R1/L1)", fmt(P));
            telemetry.addData("D (R3/L3)", String.format("%.4f", D));
            telemetry.addData("F (R2/L2)", fmt(F));
            telemetry.addData("Power cap (dpad up/down)", fmt(POWER_CAP));
            telemetry.addData("Velocity", velocity);
            telemetry.addData("Loop time", dt * 1000.0);
            telemetry.update();
        }
        turretMotor.set(0);
    }

    private void readTurret() {
        turretMotor.readPosition();
        turretMotor.readVelocity();
        currentPos = turretMotor.getScaledPos();
        velocity = turretMotor.getVelocity() / turretMotor.getTicksPerRev() * 360.0 * gearRatio;
        predictedPos = currentPos + velocity * LOOKAHEAD_S;
    }

    private void readButtons() {
        if (gamepad1.a && !lastA) {
            motorOn = !motorOn;
            target = currentPos;
        }
        if (gamepad1.dpad_right && !lastRight) target -= 5;
        if (gamepad1.dpad_left && !lastLeft) target += 5;
        if (gamepad1.y && !lastY) target = -90;
        if (gamepad1.b && !lastB) target = MIN_ANGLE + TARGET_MARGIN;
        if (gamepad1.x && !lastX) target = MAX_ANGLE - TARGET_MARGIN;
        if (gamepad1.dpad_up && !lastUp) POWER_CAP = Range.clip(POWER_CAP + 0.05, 0.05, 1.0);
        if (gamepad1.dpad_down && !lastDown) POWER_CAP = Range.clip(POWER_CAP - 0.05, 0.05, 1.0);
        if (gamepad1.right_bumper && !lastRB) P += 0.002;
        if (gamepad1.left_bumper && !lastLB) P = Math.max(0, P - 0.002);
        if (gamepad1.right_stick_button && !lastR3) D += 0.0005;
        if (gamepad1.left_stick_button && !lastL3) D = Math.max(0, D - 0.0005);
        boolean rt = gamepad1.right_trigger > 0.5, lt = gamepad1.left_trigger > 0.5;
        if (rt && !lastRT) F += 0.02;
        if (lt && !lastLT) F = Math.max(0, F - 0.02);

        lastA = gamepad1.a;
        lastB = gamepad1.b;
        lastX = gamepad1.x;
        lastY = gamepad1.y;
        lastUp = gamepad1.dpad_up;
        lastDown = gamepad1.dpad_down;
        lastLeft = gamepad1.dpad_left;
        lastRight = gamepad1.dpad_right;
        lastRB = gamepad1.right_bumper;
        lastLB = gamepad1.left_bumper;
        lastRT = rt;
        lastLT = lt;
        lastR3 = gamepad1.right_stick_button;
        lastL3 = gamepad1.left_stick_button;
    }

    private void checkFaults() {
        if (currentPos > MAX_ANGLE + FAULT_MARGIN || currentPos < MIN_ANGLE - FAULT_MARGIN) {
            fault(String.format("past limit at %.1f, if it was turning the wrong way restart and press Square during init", currentPos));
        }

        double ticks = turretMotor.readandGetPosTicks();
        if (Math.abs(lastPower) >= CHECK_POWER && ticks == lastTicks) {
            stuckCount++;
            if (stuckCount >= STUCK_LOOPS) {
                fault(String.format("encoder not moving at power %.2f, stuck or unplugged", lastPower));
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

        if (faulted) {
            power = 0;
            limitedBy = "fault";
        }
        return power;
    }

    private String status() {
        if (faulted) return "Faulted, restart to clear";
        if (currentPos > MAX_ANGLE) return "Past back limit by " + fmt(currentPos - MAX_ANGLE);
        if (currentPos < MIN_ANGLE) return "Past right limit by " + fmt(MIN_ANGLE - currentPos);
        if (MAX_ANGLE - currentPos < SLOW_ZONE) return "Slow zone (back), max power " + fmt(zoneCap(MAX_ANGLE - currentPos));
        if (currentPos - MIN_ANGLE < SLOW_ZONE) return "Slow zone (right), max power " + fmt(zoneCap(currentPos - MIN_ANGLE));
        return "OK";
    }

    private double zoneCap(double distToLimit) {
        if (distToLimit <= 0) return 0;
        if (distToLimit >= SLOW_ZONE) return POWER_CAP;
        return MIN_SLOW_POWER + (POWER_CAP - MIN_SLOW_POWER) * (distToLimit / SLOW_ZONE);
    }

    private void fault(String reason) {
        if (!faulted) {
            faulted = true;
            faultReason = reason;
        }
    }

    private static String fmt(double v) {
        return String.format("%.3f", v);
    }
}
