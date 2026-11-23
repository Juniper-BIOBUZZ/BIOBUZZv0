package org.firstinspires.ftc.teamcode.Launcher.testClasses;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Core.NewIntakeTransferHWmap;

@TeleOp(group = "Launcher", name = "Flywheel")
@Config
public class OldFlywheelTest extends LinearOpMode {
    public FlywheelTest flywheel;
    public NewIntakeTransferHWmap hwMap;

    @Override
    public void runOpMode() {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        hwMap = new NewIntakeTransferHWmap(hardwareMap);
        flywheel = new FlywheelTest(hwMap, telemetry);

        waitForStart();

        while (opModeIsActive()) {
            flywheel.update();
            telemetry.update();
        }
    }
}
