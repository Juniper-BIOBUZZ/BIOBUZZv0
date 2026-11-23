package org.firstinspires.ftc.teamcode.Launcher.testClasses;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Core.HWMap;

/*
 * Hand test (servo off):
 *   hood all the way down, then press start
 *   move the hood up by hand, the angle should go up
 *   if it goes negative, restart and press Circle during init
 *   push it all the way up and write down the angle, that is the top limit (MAX_ANGLE)
 *
 * Servo test:
 *   press Cross to turn the servo on, then dpad up a few degrees, the hood should go up
 *   if it goes the wrong way it will fault, restart and press Square during init
 *   R1/L1 change P until it gets to the target without bouncing
 *   R2/L2 add F if it stops short
 *   Triangle goes to the top, Square goes to the bottom
 */
@TeleOp(group = "Launcher", name = "Pitch Bottom/Top Test")
public class OldLauncherServoTest extends LinearOpMode {

    public static double gearRatio = 3.0 / 29.0;
    public static double MIN_ANGLE = 0;
    public static double MAX_ANGLE = 30;
    public static double FAULT_MARGIN = 3;

    public static double P = 0.03, I = 0, D = 0, F = 0.05;
    public static double TOLERANCE = 0.5;
    public static double POWER_CAP = 0.6;

    public static double CHECK_POWER = 0.3;
    public static double CHECK_TIME = 0.25;
    public static double CHECK_MIN_MOVE = 0.3;

    private CRServo pitchServo;
    private AnalogInput pitchEncoder;
    private PIDController pid;
    private final ElapsedTime loopTimer = new ElapsedTime();
    private final ElapsedTime checkTimer = new ElapsedTime();

    private boolean invertPower = false, invertEncoder = false, servoOn = false;
    private double target = 0;
    private double lastRaw = 0, servoDeg = 0, hoodAngle = 0;
    private double lastPower = 0;
    private boolean checking = false;
    private double checkStartAngle = 0;
    private boolean faulted = false;
    private String faultReason = "none";
    private String limitedBy = "none";

    private boolean lastA, lastX, lastB, lastY, lastUp, lastDown, lastRB, lastLB, lastRT, lastLT;

    @Override
    public void runOpMode() throws InterruptedException {
        HWMap hwMap = new HWMap(hardwareMap);
        pitchServo = hwMap.getPitchServo();
        pitchEncoder = hwMap.getPitchEncoder();
        pitchServo.set(0);

        while (!isStarted() && !isStopRequested()) {
            if (gamepad1.x && !lastX) invertPower = !invertPower;
            if (gamepad1.b && !lastB) invertEncoder = !invertEncoder;
            lastX = gamepad1.x;
            lastB = gamepad1.b;

            telemetry.addLine("Hood all the way down before pressing start");
            telemetry.addData("Flip servo (Square)", invertPower);
            telemetry.addData("Flip encoder (Circle)", invertEncoder);
            telemetry.addData("Raw encoder deg", rawAngle());
            telemetry.update();
        }
        if (isStopRequested()) return;

        lastRaw = rawAngle();
        servoDeg = 0;
        hoodAngle = 0;
        target = MIN_ANGLE;
        pid = new PIDController(P, I, D);
        loopTimer.reset();

        while (opModeIsActive()) {
            double dt = loopTimer.seconds();
            loopTimer.reset();

            readHood();
            readButtons();
            checkFaults();

            double power = 0;
            double error = target - hoodAngle;
            limitedBy = "none";

            if (servoOn) {
                pid.setPID(P, I, D);
                pid.setTolerance(TOLERANCE);
                power = pid.calculate(hoodAngle, target);
                if (Math.abs(error) > TOLERANCE) {
                    power += F * Math.signum(error);
                }
                if (Math.abs(power) > POWER_CAP) {
                    power = Math.signum(power) * POWER_CAP;
                    limitedBy = "power cap";
                }
            }

            if (power > 0 && hoodAngle >= MAX_ANGLE) {
                power = 0;
                limitedBy = "top limit";
            }
            if (power < 0 && hoodAngle <= MIN_ANGLE) {
                power = 0;
                limitedBy = "bottom limit";
            }
            if (faulted) {
                power = 0;
                limitedBy = "fault";
            }

            pitchServo.set(invertPower ? -power : power);
            lastPower = power;

            telemetry.addData("Faulted", faulted ? "YES - " + faultReason : "no");
            telemetry.addData("Servo (Cross)", servoOn ? "ON" : "off");
            telemetry.addData("Target (dpad, Triangle top, Square bottom)", target);
            telemetry.addData("Hood angle", hoodAngle);
            telemetry.addData("Error", error);
            telemetry.addData("Power", power);
            telemetry.addData("Limited by", limitedBy);
            telemetry.addData("P (R1/L1)", fmt(P));
            telemetry.addData("F (R2/L2)", fmt(F));
            telemetry.addData("Servo deg from bottom", servoDeg);
            telemetry.addData("Raw encoder deg", lastRaw);
            telemetry.addData("Encoder voltage", pitchEncoder.getVoltage());
            telemetry.addData("Flip servo / encoder", invertPower + " / " + invertEncoder);
            telemetry.addData("Loop time", dt * 1000.0);
            telemetry.update();
        }
        pitchServo.set(0);
    }

    private double rawAngle() {
        double raw = pitchEncoder.getVoltage() / 3.3 * 360.0;
        return invertEncoder ? 360.0 - raw : raw;
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

    private void readButtons() {
        if (gamepad1.a && !lastA) {
            servoOn = !servoOn;
            target = Range.clip(hoodAngle, MIN_ANGLE, MAX_ANGLE);
        }
        if (gamepad1.dpad_up && !lastUp) target += 1;
        if (gamepad1.dpad_down && !lastDown) target -= 1;
        if (gamepad1.y && !lastY) target = MAX_ANGLE - 2;
        if (gamepad1.x && !lastX) target = MIN_ANGLE;
        if (gamepad1.right_bumper && !lastRB) P += 0.005;
        if (gamepad1.left_bumper && !lastLB) P = Math.max(0, P - 0.005);
        boolean rt = gamepad1.right_trigger > 0.5, lt = gamepad1.left_trigger > 0.5;
        if (rt && !lastRT) F += 0.01;
        if (lt && !lastLT) F = Math.max(0, F - 0.01);

        target = Range.clip(target, MIN_ANGLE, MAX_ANGLE);

        lastA = gamepad1.a;
        lastX = gamepad1.x;
        lastY = gamepad1.y;
        lastUp = gamepad1.dpad_up;
        lastDown = gamepad1.dpad_down;
        lastRB = gamepad1.right_bumper;
        lastLB = gamepad1.left_bumper;
        lastRT = rt;
        lastLT = lt;
    }

    private void checkFaults() {
        if (hoodAngle < MIN_ANGLE - FAULT_MARGIN) {
            fault(String.format("went below the bottom (%.1f), servo and encoder probably disagree on direction. Restart and press Circle during init", hoodAngle));
        }
        if (hoodAngle > MAX_ANGLE + FAULT_MARGIN) {
            fault(String.format("past the top limit (%.1f), check MAX_ANGLE", hoodAngle));
        }

        if (Math.abs(lastPower) >= CHECK_POWER) {
            if (!checking) {
                checking = true;
                checkStartAngle = hoodAngle;
                checkTimer.reset();
            } else if (checkTimer.seconds() >= CHECK_TIME) {
                if (Math.abs(hoodAngle - checkStartAngle) < CHECK_MIN_MOVE) {
                    fault(String.format("hood not moving at power %.2f, stuck, unplugged encoder, or too little power", lastPower));
                }
                checkStartAngle = hoodAngle;
                checkTimer.reset();
            }
        } else {
            checking = false;
        }
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