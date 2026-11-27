package org.firstinspires.ftc.teamcode.Test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Core.HWMap;
import org.firstinspires.ftc.teamcode.Core.Logger;
import org.firstinspires.ftc.teamcode.Core.MotorWrapper;
import org.firstinspires.ftc.teamcode.Launcher.FlywheelFSM;
import org.firstinspires.ftc.teamcode.Launcher.PitchFSM;
import org.firstinspires.ftc.teamcode.Launcher.TurretFSM;
import org.firstinspires.ftc.teamcode.Transfer.GateFSM;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Locale;

@Config
@TeleOp(group = "Launcher", name = "Interpolation Test")
public class IntegratedMechanismTuningTest extends LinearOpMode {

    public static boolean ARM = false;
    public static boolean ENABLE_TURRET = false;
    public static boolean ENABLE_PITCH = false;
    public static boolean ENABLE_FLYWHEEL = false;
    public static boolean ENABLE_INTAKE = false;
    public static boolean ENABLE_TRANSFER = false;
    public static boolean ENABLE_GATE = false;

    public static double TURRET_TARGET_DEG = -4;

    public static double PITCH_TARGET_POSITION = 0.0;
    public static double PITCH_MIN_POSITION = 0.0;
    public static double PITCH_MAX_POSITION = 1.0;

    public static boolean ENABLE_PITCH_COMPENSATION = false;
    public static double FLYWHEEL_ERROR_THRESHOLD = 40;
    public static double PITCH_REDUCTION_FACTOR = 0.0005;
    public static double PITCH_COMPENSATION_SIGN = 1.0;

    public static double FLYWHEEL_TARGET_RPM = 0;
    public static double TRANSFER_TARGET_RPM = 0;
    public static double GATE_TARGET_POSITION = 0.43;

    public static double MAX_FLYWHEEL_RPM = 4000;
    public static double MAX_TRANSFER_RPM = 312;
    public static double GATE_MIN_POSITION = 0.16;
    public static double GATE_MAX_POSITION = 0.43;

    public static final double INTAKE_RATED_RPM = 1150;

    public static boolean LOG_CSV = false;
    public static double LOG_INTERVAL_MS = 50;
    public static double DISTANCE_IN = 0;
    public static int TRIAL = 1;

    private BufferedWriter csvWriter;
    private String csvStatus = "OFF";
    private double lastLogMs = 0;
    private double lastFlushMs = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        ARM = false;
        ENABLE_TURRET = false;
        ENABLE_PITCH = false;
        ENABLE_FLYWHEEL = false;
        ENABLE_INTAKE = false;
        ENABLE_TRANSFER = false;
        ENABLE_GATE = false;
        LOG_CSV = false;

        telemetry = new MultipleTelemetry(
                telemetry,
                FtcDashboard.getInstance().getTelemetry()
        );

        Logger logger = new Logger(telemetry);
        HWMap hwMap = new HWMap(hardwareMap);

        TurretFSM turret = new TurretFSM(hwMap, logger);
        PitchFSM pitch = new PitchFSM(hwMap, logger);
        FlywheelFSM flywheel = new FlywheelFSM(hwMap, logger);

        MotorWrapper intake = new MotorWrapper(
                hwMap.getIntakeMotor(),
                false,
                1,
                false
        );

        hwMap.getIntakeMotor().motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hwMap.getIntakeMotor().motor.setDirection(DcMotorSimple.Direction.REVERSE);

        MotorWrapper transfer = new MotorWrapper(
                hwMap.getTransferMotor(),
                true,
                1,
                false
        );

        hwMap.getIntakeMotor().stopMotor();
        hwMap.getTransferMotor().stopMotor();
        hwMap.getTransferServo().setPosition(GateFSM.positionDown);
        flywheel.setTargetRPM(0);

        while (opModeInInit()) {
            telemetry.addLine("Park turret at its zero position BEFORE INIT.");
            telemetry.addLine("Pitch uses Axon Servo mode: 0.0 = down, 1.0 = up.");
            telemetry.addLine("Dashboard: set ARM=true, then enable each mechanism individually.");
            telemetry.addLine("No Pinpoint targeting or drivetrain in this test.");
            telemetry.update();
            idle();
        }

        if (isStopRequested()) {
            return;
        }

        pitch.setTargetPosition(0.0);
        pitch.updateState();
        hwMap.getPitchServo().setPosition(0.0);

        ElapsedTime runtime = new ElapsedTime();
        runtime.reset();

        try {
            while (opModeIsActive()) {

                boolean turretOn = ARM && ENABLE_TURRET;
                boolean pitchOn = ARM && ENABLE_PITCH;
                boolean flywheelOn = ARM && ENABLE_FLYWHEEL;
                boolean intakeOn = ARM && ENABLE_INTAKE;
                boolean transferOn = ARM && ENABLE_TRANSFER;
                boolean gateOn = (ARM && ENABLE_GATE) || gamepad1.right_trigger > 0.5;

                if (turretOn) {
                    turret.setTargetAngle(
                            safeClip(
                                    TURRET_TARGET_DEG,
                                    TurretFSM.minTarget(),
                                    TurretFSM.maxTarget(),
                                    TurretFSM.maxTarget()
                            )
                    );
                }

                turret.updateState(turretOn);

                double flywheelTarget = flywheelOn
                        ? safeClip(
                        FLYWHEEL_TARGET_RPM,
                        0,
                        Math.max(0, Double.isFinite(MAX_FLYWHEEL_RPM) ? MAX_FLYWHEEL_RPM : 0),
                        0
                )
                        : 0;

                flywheel.setTargetRPM(flywheelTarget);
                flywheel.updateState();

                double flywheelActual = flywheel.getCurrentRPM();
                double flywheelError = flywheelTarget - flywheelActual;

                double basePitchTarget = pitchOn
                        ? safeClip(
                        PITCH_TARGET_POSITION,
                        PITCH_MIN_POSITION,
                        PITCH_MAX_POSITION,
                        PITCH_MIN_POSITION
                )
                        : PITCH_MIN_POSITION;

                double pitchCompensation = 0.0;

                if (pitchOn
                        && flywheelOn
                        && ENABLE_PITCH_COMPENSATION
                        && flywheelError > FLYWHEEL_ERROR_THRESHOLD) {
                    pitchCompensation = flywheelError
                            * PITCH_REDUCTION_FACTOR
                            * PITCH_COMPENSATION_SIGN;
                }

                double pitchTarget = safeClip(
                        basePitchTarget + pitchCompensation,
                        PITCH_MIN_POSITION,
                        PITCH_MAX_POSITION,
                        PITCH_MIN_POSITION
                );

                if (pitchOn) {
                    pitch.setTargetPosition(pitchTarget);
                    pitch.updateState();
                } else {
                    hwMap.getPitchServo().setPosition(PITCH_MIN_POSITION);
                }

                double intakeTarget = intakeOn ? INTAKE_RATED_RPM : 0;

                hwMap.getIntakeMotor().motor.setPower(intakeOn ? 1.0 : 0.0);
                intake.readVelocity();

                double transferLimit = Double.isFinite(MAX_TRANSFER_RPM)
                        ? Math.abs(MAX_TRANSFER_RPM)
                        : 0;

                double transferTarget = transferOn
                        ? safeClip(
                        TRANSFER_TARGET_RPM,
                        -transferLimit,
                        transferLimit,
                        0
                )
                        : 0;

                transfer.readVelocity();

                if (transferOn && transferTarget != 0) {
                    transfer.setRPM(transferTarget);
                } else {
                    hwMap.getTransferMotor().stopMotor();
                }

                double gateCommand = gateOn
                        ? GateFSM.positionUp
                        : GateFSM.positionDown;

                hwMap.getTransferServo().setPosition(gateCommand);

                double turretTarget = turret.getTargetAngle();
                double turretActual = turret.getCurrentAngle();

                double pitchActual = pitch.getCurrentPosition();

                double intakeActual = intake.getRPM();
                double transferActual = transfer.getRPM();
                double timeMs = runtime.milliseconds();

                double turretAmps = hwMap.getTurretMotor()
                        .motorEx.getCurrent(CurrentUnit.AMPS);

                double flywheelAmps = hwMap.getFlywheelMotor()
                        .motorEx.getCurrent(CurrentUnit.AMPS);

                double intakeAmps = intake.getCurrent();
                double transferAmps = transfer.getCurrent();

                telemetry.addData("MASTER ARM", ARM);

                telemetry.addData(
                        "Turret enabled / fault",
                        turretOn + " / " + (
                                turret.FAULTED()
                                        ? turret.getFaultReason()
                                        : "none"
                        )
                );

                telemetry.addData(
                        "Turret target / actual / error (deg)",
                        "%.1f / %.1f / %.1f",
                        turretTarget,
                        turretActual,
                        turretTarget - turretActual
                );

                telemetry.addData(
                        "Pitch enabled / fault",
                        pitchOn + " / " + (
                                pitch.FAULTED()
                                        ? pitch.getFaultReason()
                                        : "none"
                        )
                );

                telemetry.addData(
                        "Pitch target / commanded position",
                        "%.3f / %.3f",
                        pitchTarget,
                        pitchActual
                );

                telemetry.addData(
                        "Pitch base / compensation / commanded",
                        "%.3f / %.3f / %.3f",
                        basePitchTarget,
                        pitchCompensation,
                        pitchTarget
                );

                telemetry.addData(
                        "Pitch compensation enabled / threshold / factor / sign",
                        "%s / %.0f / %.6f / %.1f",
                        ENABLE_PITCH_COMPENSATION,
                        FLYWHEEL_ERROR_THRESHOLD,
                        PITCH_REDUCTION_FACTOR,
                        PITCH_COMPENSATION_SIGN
                );

                telemetry.addData(
                        "Pitch encoder angle",
                        "%.1f",
                        pitch.getEncoderAngle()
                );

                telemetry.addData("Flywheel enabled", flywheelOn);

                telemetry.addData("Flywheel Target RPM", flywheelTarget);
                telemetry.addData("Flywheel Current RPM", flywheelActual);
                telemetry.addData("Flywheel Error RPM", flywheelError);

                telemetry.addData("Intake enabled", intakeOn);

                telemetry.addData(
                        "Intake target / actual RPM",
                        "%.0f / %.0f",
                        intakeTarget,
                        intakeActual
                );

                telemetry.addData("Transfer enabled", transferOn);

                telemetry.addData(
                        "Transfer target / actual RPM",
                        "%.0f / %.0f",
                        transferTarget,
                        transferActual
                );

                telemetry.addData(
                        "Gate enabled / commanded",
                        "%s / %.3f",
                        gateOn,
                        gateCommand
                );

                telemetry.addData(
                        "Motor current (turret/flywheel/intake/transfer) A",
                        "%.2f / %.2f / %.2f / %.2f",
                        turretAmps,
                        flywheelAmps,
                        intakeAmps,
                        transferAmps
                );

                telemetry.addData(
                        "Data collection distance / trial",
                        "%.1f in / %d",
                        DISTANCE_IN,
                        TRIAL
                );

                telemetry.addData("CSV", csvStatus);

                updateCsv(
                        timeMs,
                        turretOn,
                        pitchOn,
                        flywheelOn,
                        intakeOn,
                        transferOn,
                        gateOn,
                        turretTarget,
                        turretActual,
                        pitchTarget,
                        pitchActual,
                        flywheelTarget,
                        flywheelActual,
                        intakeTarget,
                        intakeActual,
                        transferTarget,
                        transferActual,
                        gateCommand,
                        turretAmps,
                        flywheelAmps,
                        intakeAmps,
                        transferAmps
                );

                telemetry.update();
                idle();
            }

        } finally {
            flywheel.setTargetRPM(0);

            hwMap.getFlywheelMotor().stopMotor();
            hwMap.getIntakeMotor().stopMotor();
            hwMap.getTransferMotor().stopMotor();
            hwMap.getTurretMotor().stopMotor();

            hwMap.getPitchServo().setPosition(PITCH_MIN_POSITION);
            hwMap.getTransferServo().setPosition(GateFSM.positionDown);

            closeCsv();
        }
    }

    private void updateCsv(
            double timeMs,
            boolean turretOn,
            boolean pitchOn,
            boolean flywheelOn,
            boolean intakeOn,
            boolean transferOn,
            boolean gateOn,
            double turretTarget,
            double turretActual,
            double pitchTarget,
            double pitchActual,
            double flywheelTarget,
            double flywheelActual,
            double intakeTarget,
            double intakeActual,
            double transferTarget,
            double transferActual,
            double gateCommand,
            double turretAmps,
            double flywheelAmps,
            double intakeAmps,
            double transferAmps
    ) {

        if (!LOG_CSV) {
            if (csvWriter != null) {
                closeCsv();
            } else {
                csvStatus = "OFF";
            }
            return;
        }

        if (csvWriter == null) {

            File folder = new File("/sdcard/FIRST");

            if (!folder.exists() && !folder.mkdirs()) {
                csvStatus = "ERROR: cannot create /sdcard/FIRST";
                LOG_CSV = false;
                return;
            }

            String stamp = new SimpleDateFormat(
                    "yyyyMMdd_HHmmss",
                    Locale.US
            ).format(new Date());

            File file = new File(
                    folder,
                    "integrated_tuning_" + stamp + ".csv"
            );

            try {

                csvWriter = new BufferedWriter(new FileWriter(file));

                csvWriter.write(
                        "time_ms,trial,distance_in,turret_on,turret_target_deg,turret_actual_deg,"
                                + "pitch_on,pitch_target_position,pitch_actual_position,"
                                + "flywheel_on,flywheel_target_rpm,flywheel_actual_rpm,"
                                + "intake_on,intake_target_ticks_s,intake_actual_ticks_s,"
                                + "transfer_on,transfer_target_rpm,transfer_actual_rpm,"
                                + "gate_on,gate_commanded_position,"
                                + "turret_amps,flywheel_amps,intake_amps,transfer_amps\n"
                );

                csvStatus = file.getAbsolutePath();
                lastLogMs = timeMs - LOG_INTERVAL_MS;
                lastFlushMs = timeMs;

            } catch (IOException e) {

                csvStatus = "ERROR opening CSV: " + e.getMessage();
                closeCsv();
                LOG_CSV = false;
                return;
            }
        }

        if (timeMs - lastLogMs < Math.max(20, LOG_INTERVAL_MS)) {
            return;
        }

        try {

            csvWriter.write(
                    String.format(
                            Locale.US,
                            "%.0f,%d,%.2f,%b,%.2f,%.2f,%b,%.3f,%.3f,"
                                    + "%b,%.1f,%.1f,%b,%.1f,%.1f,%b,%.1f,%.1f,"
                                    + "%b,%.3f,%.3f,%.3f,%.3f,%.3f%n",
                            timeMs,
                            TRIAL,
                            DISTANCE_IN,
                            turretOn,
                            turretTarget,
                            turretActual,
                            pitchOn,
                            pitchTarget,
                            pitchActual,
                            flywheelOn,
                            flywheelTarget,
                            flywheelActual,
                            intakeOn,
                            intakeTarget,
                            intakeActual,
                            transferOn,
                            transferTarget,
                            transferActual,
                            gateOn,
                            gateCommand,
                            turretAmps,
                            flywheelAmps,
                            intakeAmps,
                            transferAmps
                    )
            );

            lastLogMs = timeMs;

            if (timeMs - lastFlushMs >= 1000) {
                csvWriter.flush();
                lastFlushMs = timeMs;
            }

        } catch (IOException e) {

            csvStatus = "ERROR writing CSV: " + e.getMessage();
            closeCsv();
            LOG_CSV = false;
        }
    }

    private static double safeClip(
            double value,
            double min,
            double max,
            double fallback
    ) {

        if (!Double.isFinite(value)
                || !Double.isFinite(min)
                || !Double.isFinite(max)) {
            return fallback;
        }

        return Range.clip(value, min, max);
    }

    private void closeCsv() {

        if (csvWriter != null) {

            try {
                csvWriter.flush();
                csvWriter.close();
            } catch (IOException ignored) {
            }

            csvWriter = null;
        }

        if (!csvStatus.startsWith("ERROR")) {
            csvStatus = "OFF";
        }
    }
}
