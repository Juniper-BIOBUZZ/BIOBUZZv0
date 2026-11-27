package org.firstinspires.ftc.teamcode.Launcher.testClasses;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Core.HWMap;
import org.firstinspires.ftc.teamcode.Core.MotorWrapper;

@Config
@TeleOp(group = "Launcher", name = "Flywheel Test")
public class FlywheelTest extends LinearOpMode {

    public static double vP = 4;
    public static double vI = 5;
    public static double vD = 0;

    public static double kS = 0;
    public static double kV = 2.6;
    public static double kA = 0;

    public static double TARGET_RPM = 1575;
    public static boolean ENABLE_FLYWHEEL = false;

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        HWMap hwMap = new HWMap(hardwareMap);
        MotorWrapper flywheelMotor = new MotorWrapper(hwMap.getFlywheelMotor(), true, 1, false);

        hwMap.getFlywheelMotor().stopMotor();

        telemetry.addLine("FlywheelTest ready.");
        telemetry.addLine("Set ENABLE_FLYWHEEL = true in Dashboard to run.");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) {
            return;
        }

        try {
            while (opModeIsActive()) {
                flywheelMotor.readVelocity();
                flywheelMotor.setVelocityConstants(vP, vI, vD, kS, kV, kA);

                double currentRPM = flywheelMotor.getRPM();
                double targetRPM = ENABLE_FLYWHEEL ? TARGET_RPM : 0;
                double errorRPM = targetRPM - currentRPM;

                if (ENABLE_FLYWHEEL && TARGET_RPM > 0) {
                    flywheelMotor.setRPM(TARGET_RPM);
                } else {
                    hwMap.getFlywheelMotor().stopMotor();
                }

                telemetry.addData("Flywheel Target RPM", targetRPM);
                telemetry.addData("Flywheel Current RPM", currentRPM);
                telemetry.addData("Flywheel Error RPM", errorRPM);

                telemetry.addData("vP", vP);
                telemetry.addData("vI", vI);
                telemetry.addData("vD", vD);
                telemetry.addData("kS", kS);
                telemetry.addData("kV", kV);
                telemetry.addData("kA", kA);

                telemetry.update();
                idle();
            }
        } finally {
            hwMap.getFlywheelMotor().stopMotor();
        }
    }
}
