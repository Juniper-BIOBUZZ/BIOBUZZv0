package org.firstinspires.ftc.teamcode.Core;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

/*
 * Run this before a match to pick the alliance:
 *   Circle = red, Cross = blue
 *   the start position goes with the alliance (touching the wall in front of the up cell)
 *   press stop to save
 */
@TeleOp(name = "Robot Settings", group = "A")
public class ConfigOpMode extends LinearOpMode {

    private boolean lastCircle, lastCross;

    @Override
    public void runOpMode() {
        RobotSettings settings = RobotSettings.load();

        telemetry.addLine("Press start, pick the alliance, then press stop to save");
        telemetry.addData("Alliance", settings.alliance);
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.b && !lastCircle) settings.alliance = RobotSettings.Alliance.RED;
            if (gamepad1.a && !lastCross) settings.alliance = RobotSettings.Alliance.BLUE;
            settings.startPosState = RobotSettings.StartPos.forAlliance(settings.alliance);

            lastCircle = gamepad1.b;
            lastCross = gamepad1.a;

            Pose2D start = settings.startPosState.getPose2D();
            telemetry.addData("Alliance (Circle red, Cross blue)", settings.alliance);
            telemetry.addData("Start position", settings.startPosState);
            telemetry.addData("Start x / y / heading", String.format("%.1f / %.1f / %.1f",
                    start.getX(DistanceUnit.INCH), start.getY(DistanceUnit.INCH), start.getHeading(AngleUnit.DEGREES)));
            telemetry.addData("Cell facing up at the start", settings.alliance.startUpCell());
            telemetry.addLine("Press stop to save");
            telemetry.update();
        }

        settings.save();
        telemetry.addLine("Saved");
        telemetry.update();
    }
}
