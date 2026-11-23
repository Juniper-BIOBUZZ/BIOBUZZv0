package org.firstinspires.ftc.teamcode.Launcher.testClasses;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Core.HWMap;
import org.firstinspires.ftc.teamcode.Core.MotorWrapper;

/*
 * Turret hardstop test (no PID)
 *
 * 0 = turret facing back (start pos, encoder gets zeroed here so park it here before init)
 * -195 = right limit (measured on the robot)
 * No mechanical stops, if it goes past these the wires get ripped
 *
 * Hand test (ENABLE_MOTOR = false):
 *   motor gets no power, just turn the turret by hand
 *   turning right should make the angle go negative
 *   at the right limit it should read about -195, if not the gear ratio is off
 *   check that the status changes when you get near/past each limit
 *
 * Motor test (ENABLE_MOTOR = true, keep POWER_CAP low):
 *   gamepad1 left stick x moves the turret, no PID
 *   push into each limit, it should slow down and then stop at the limit
 *   if stick right turns it left, flip INVERT_MOTOR and restart
 *
 * What the hardstops do:
 *   slow zone  - lowers max power when the turret is getting close to a limit
 *                (uses predicted pos = pos + velocity * LOOKAHEAD_S)
 *   gate       - at or past a limit, no power going further out
 *   fault      - more than FAULT_MARGIN past a limit kills the turret until restart
 *   encoder    - if we give it power and the encoder does not change for a few loops, fault
 *   slew       - power ramps up instead of jumping (going to 0 is instant)
 *   brake mode - so it stops instead of coasting
 */
@Config
@TeleOp(group = "Launcher", name = "Turret Hardstop Test")
public class TurretHardstopTest extends LinearOpMode {

    // test settings
    public static boolean ENABLE_MOTOR = true;  // false = turn by hand, motor stays off
    public static double POWER_CAP = 0.3;        // max power from the stick
    public static boolean INVERT_MOTOR = false;
    public static double gearRatio = 16.0 / 109.0;

    // hardstops (degrees)
    public static double MAX_ANGLE = 0;          // back
    public static double MIN_ANGLE = -195;       // right limit
    public static double FAULT_MARGIN = 5;       // how far past a limit before it faults
    public static double SLOW_ZONE = 30;         // starts slowing down this far from a limit
    public static double MIN_SLOW_POWER = 0.1;   // max power right at the limit
    public static double LOOKAHEAD_S = 0.06;     // seconds ahead for predicted pos

    // encoder check
    public static double CHECK_POWER = 0.25;     // only checks when power is above this
    public static int STUCK_LOOPS = 3;           // loops with no encoder change before fault

    // slew
    public static double POWER_SLEW = 4.0;       // how fast power can go up (per second)
    private static final double MAX_SLEW_DT = 0.05;

    private MotorWrapper turretMotor;
    private final ElapsedTime loopTimer = new ElapsedTime();

    private double currentPos = 0, velocity = 0, predictedPos = 0;
    private double lastPower = 0, lastTicks = 0;
    private int stuckCount = 0;
    private boolean faulted = false;
    private String faultReason = "none";
    private String limitedBy = "none";

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        HWMap hwMap = new HWMap(hardwareMap);
        hwMap.getTurretMotor().setInverted(INVERT_MOTOR);
        hwMap.getTurretMotor().setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        turretMotor = new MotorWrapper(hwMap.getTurretMotor(), false, gearRatio, false);
        turretMotor.resetEncoder();
        turretMotor.set(0);
        lastTicks = turretMotor.readandGetPosTicks();

        telemetry.addLine("Park turret facing back before init");
        telemetry.addLine(ENABLE_MOTOR ? "Motor on, use left stick x" : "Motor off, turn turret by hand");
        telemetry.update();

        waitForStart();
        loopTimer.reset();

        while (opModeIsActive()) {
            double dt = loopTimer.seconds();
            loopTimer.reset();

            readTurret();
            checkFaults();
            double requested = ENABLE_MOTOR ? -gamepad1.left_stick_x * POWER_CAP : 0;
            double power = applyHardstops(requested, dt);

            turretMotor.set(power);
            lastPower = power;

            telemetry.addData("Fault", faulted ? faultReason : "none");
            telemetry.addData("Motor on", ENABLE_MOTOR);
            telemetry.addData("Status", status());
            telemetry.addData("Angle", currentPos);
            telemetry.addData("Predicted angle", predictedPos);
            telemetry.addData("To back limit", MAX_ANGLE - currentPos);
            telemetry.addData("To right limit", currentPos - MIN_ANGLE);
            telemetry.addData("Velocity", velocity);
            telemetry.addData("Stick power", requested);
            telemetry.addData("Actual power", power);
            telemetry.addData("Limited by", limitedBy);
            telemetry.addData("Ticks", turretMotor.readandGetPosTicks());
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

    private void checkFaults() {
        if (currentPos > MAX_ANGLE + FAULT_MARGIN || currentPos < MIN_ANGLE - FAULT_MARGIN) {
            fault(String.format("past limit at %.1f", currentPos));
        }

        double ticks = turretMotor.readandGetPosTicks();
        if (Math.abs(lastPower) >= CHECK_POWER && ticks == lastTicks) {
            stuckCount++;
            if (stuckCount >= STUCK_LOOPS) {
                fault("encoder not moving");
            }
        } else {
            stuckCount = 0;
        }
        lastTicks = ticks;
    }

    private double applyHardstops(double requested, double dt) {
        double power = Range.clip(requested, -POWER_CAP, POWER_CAP);
        limitedBy = "none";

        // slew
        double step = POWER_SLEW * Math.min(dt, MAX_SLEW_DT);
        double maxMagnitude = (Math.signum(power) == Math.signum(lastPower))
                ? Math.abs(lastPower) + step
                : step;
        if (Math.abs(power) > maxMagnitude) {
            power = Math.signum(power) * maxMagnitude;
            limitedBy = "slew";
        }

        // slow zone
        double cap = POWER_CAP;
        if (power > 0) cap = zoneCap(MAX_ANGLE - predictedPos);
        if (power < 0) cap = zoneCap(predictedPos - MIN_ANGLE);
        if (Math.abs(power) > cap) {
            power = Math.signum(power) * cap;
            limitedBy = "slow zone";
        }

        // gate
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
        if (currentPos == MAX_ANGLE) return "At back limit";
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
        return String.format("%.2f", v);
    }
}
