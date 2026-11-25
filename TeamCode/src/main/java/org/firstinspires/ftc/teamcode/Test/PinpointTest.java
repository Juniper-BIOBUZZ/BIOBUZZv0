
package org.firstinspires.ftc.teamcode.Test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import org.firstinspires.ftc.teamcode.Core.Drivetrain;
import org.firstinspires.ftc.teamcode.Core.MechanumHardwareMap;
import org.firstinspires.ftc.teamcode.Core.Pinpoint;
import org.firstinspires.ftc.teamcode.Core.RobotSettings;

@TeleOp(group = "Drivetrain", name = "Pinpoint")
public class PinpointTest extends LinearOpMode {

    @Override
    public void runOpMode() {

        telemetry = new MultipleTelemetry(
                telemetry,
                FtcDashboard.getInstance().getTelemetry()
        );

        RobotSettings settings = RobotSettings.load();

        MechanumHardwareMap hwMap =
                new MechanumHardwareMap(hardwareMap);

        Pinpoint pinpoint = new Pinpoint(hwMap);
        double startHeading =
                settings.alliance == RobotSettings.Alliance.RED
                        ? 180 : 0;

        pinpoint.setPosition(
                new Pose2D(
                        DistanceUnit.INCH,
                        0,
                        0,
                        AngleUnit.DEGREES,
                        startHeading
                )
        );
        double allianceForward =
                settings.alliance == RobotSettings.Alliance.RED
                        ? -90 : 90;

        final double[] driveHeading = {startHeading};
        double headingOffset = 0;

        Drivetrain drivetrain = new Drivetrain(
                telemetry,
                gamepad1,
                hwMap,
                () -> driveHeading[0],
                settings.alliance
        );

        drivetrain.setFieldCentric(true);
        while (opModeInInit()) {

            pinpoint.update();

            telemetry.addData("Alliance", settings.alliance);
            telemetry.addData("Fixed driver forward",
                    "%.0f deg", allianceForward);

            telemetry.addData("X (in)",
                    "%.2f", pinpoint.getX());
            telemetry.addData("Y (in)",
                    "%.2f", pinpoint.getY());
            telemetry.addData("Heading (deg)",
                    "%.2f", pinpoint.getHeading());

            telemetry.addData("isPinpointReady",
                    "READY".equals(pinpoint.getStatus()));

            telemetry.addData("Pinpoint status",
                    pinpoint.getStatus());

            telemetry.addData("Start pose set",
                    pinpoint.positionSet());

            telemetry.update();
            idle();
        }

        if (isStopRequested()) return;

        boolean lastOptions = false;

        try {
            while (opModeIsActive()) {

                pinpoint.update();

                boolean isPinpointReady =
                        "READY".equals(pinpoint.getStatus());

                boolean valid =
                        isPinpointReady
                                && pinpoint.positionSet()
                                && Double.isFinite(pinpoint.getX())
                                && Double.isFinite(pinpoint.getY())
                                && Double.isFinite(pinpoint.getHeading());
                if (valid) {
                    driveHeading[0] =
                            AngleUnit.normalizeDegrees(
                                    pinpoint.getHeading()
                                            + headingOffset
                            );
                }
                if (gamepad1.dpad_up) {
                    drivetrain.setFieldCentric(true);
                }

                if (gamepad1.dpad_down) {
                    drivetrain.setFieldCentric(false);
                }
                boolean options = gamepad1.start;

                if (valid && options && !lastOptions) {

                    headingOffset =
                            AngleUnit.normalizeDegrees(
                                    allianceForward
                                            - pinpoint.getHeading()
                            );

                    driveHeading[0] = allianceForward;
                }

                lastOptions = options;


                if (valid || !drivetrain.isFieldCentric()) {
                    drivetrain.update();
                    hwMap.getFrontLeftMotor().setPower(
                            hwMap.getFrontLeftMotor().getPower() * 0.5);

                    hwMap.getFrontRightMotor().setPower(
                            hwMap.getFrontRightMotor().getPower() * 0.5);

                    hwMap.getBackLeftMotor().setPower(
                            hwMap.getBackLeftMotor().getPower() * 0.5);

                    hwMap.getBackRightMotor().setPower(
                            hwMap.getBackRightMotor().getPower() * 0.5);

                } else {
                    stopDrive(hwMap);
                }

                telemetry.addData("Alliance",
                        settings.alliance);

                telemetry.addData("Fixed driver forward",
                        "%.0f deg", allianceForward);

                telemetry.addData("X (in)",
                        "%.2f", pinpoint.getX());

                telemetry.addData("Y (in)",
                        "%.2f", pinpoint.getY());

                telemetry.addData("Heading (deg)",
                        "%.2f", pinpoint.getHeading());

                telemetry.addData("isPinpointReady",
                        isPinpointReady);

                telemetry.addData("Pinpoint status",
                        pinpoint.getStatus());

                telemetry.addData("Start pose set",
                        pinpoint.positionSet());

                telemetry.addData("Pinpoint valid",
                        valid);

                telemetry.addData("Bad reads",
                        pinpoint.getBadReads());

                telemetry.addData("Jump count",
                        pinpoint.getJumpCount());

                telemetry.addData("Last jump",
                        pinpoint.getLastJump());

                telemetry.addData("Drive heading",
                        "%.2f", driveHeading[0]);

                telemetry.addData("Heading offset",
                        "%.2f", headingOffset);

                telemetry.addData("Drive mode",
                        drivetrain.isFieldCentric()
                                ? "FIELD CENTRIC"
                                : "ROBOT CENTRIC");

                telemetry.update();
            }

        } finally {
            stopDrive(hwMap);
        }
    }

    private void stopDrive(MechanumHardwareMap hwMap) {

        hwMap.getFrontLeftMotor().setPower(0);
        hwMap.getFrontRightMotor().setPower(0);
        hwMap.getBackLeftMotor().setPower(0);
        hwMap.getBackRightMotor().setPower(0);
    }
}
