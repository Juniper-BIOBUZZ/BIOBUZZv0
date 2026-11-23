
package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Core.Drivetrain;
import org.firstinspires.ftc.teamcode.Core.MechanumHardwareMap;
import org.firstinspires.ftc.teamcode.Core.Pinpoint;
import org.firstinspires.ftc.teamcode.Core.RobotSettings;

@TeleOp(group = "Drivetrain", name = "Drivetrain Pinpoint Test")
public class DrivetrainPinpointTest extends LinearOpMode {

    private double fieldHeading = 0;
    private double headingOffset = 0;

    @Override
    public void runOpMode() {
        RobotSettings settings = RobotSettings.load();
        MechanumHardwareMap hwMap = new MechanumHardwareMap(hardwareMap);
        Pinpoint pinpoint = new Pinpoint(hwMap);
        Pose2D start = RobotSettings.StartPos.forAlliance(settings.alliance).getPose2D();
        pinpoint.setPosition(start);

        double allianceForward = settings.alliance == RobotSettings.Alliance.RED ? -90 : 90;
        Drivetrain drivetrain = new Drivetrain(telemetry, gamepad1, hwMap,
                () -> fieldHeading, settings.alliance);
        drivetrain.setFieldCentric(true);

        while (opModeInInit()) {
            pinpoint.update();
            telemetry.addData("Alliance", settings.alliance);
            telemetry.addData("Start pose set", pinpoint.positionSet());
            telemetry.addData("Pinpoint status", pinpoint.getStatus());
            telemetry.addData("Expected start X/Y/Heading", "%.1f / %.1f / %.1f",
                    start.getX(DistanceUnit.INCH), start.getY(DistanceUnit.INCH),
                    start.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Pinpoint X/Y/Heading", "%.1f / %.1f / %.1f",
                    pinpoint.getX(), pinpoint.getY(), pinpoint.getHeading());
            telemetry.addData("Bad reads", pinpoint.getBadReads());
            telemetry.update();
            idle();
        }

        if (isStopRequested()) return;

        boolean lastSquare = gamepad1.x;

        while (opModeIsActive()) {
            pinpoint.update();
            boolean valid = pinpoint.positionSet()
                    && "READY".equals(pinpoint.getStatus())
                    && Double.isFinite(pinpoint.getHeading())
                    && Double.isFinite(pinpoint.getX())
                    && Double.isFinite(pinpoint.getY());

            if (gamepad1.dpad_up) drivetrain.setFieldCentric(true);
            else if (gamepad1.dpad_down) drivetrain.setFieldCentric(false);

            if (valid) {
                fieldHeading = AngleUnit.normalizeDegrees(pinpoint.getHeading() + headingOffset);
                if (gamepad1.x && !lastSquare) {
                    headingOffset = AngleUnit.normalizeDegrees(allianceForward - pinpoint.getHeading());
                    fieldHeading = allianceForward;
                }
            }
            lastSquare = gamepad1.x;

            if (valid || !drivetrain.isFieldCentric()) drivetrain.update();
            else {
                hwMap.getFrontLeftMotor().setPower(0);
                hwMap.getFrontRightMotor().setPower(0);
                hwMap.getBackLeftMotor().setPower(0);
                hwMap.getBackRightMotor().setPower(0);
            }

            telemetry.addData("Mode (Dpad Up FC / Down RC)",
                    drivetrain.isFieldCentric() ? "FIELD CENTRIC" : "ROBOT CENTRIC");
            telemetry.addData("Pinpoint valid for FC", valid);
            telemetry.addData("Pinpoint status", pinpoint.getStatus());
            telemetry.addData("Start pose set", pinpoint.positionSet());
            telemetry.addData("Pinpoint X/Y", "%.1f / %.1f",
                    pinpoint.getX(), pinpoint.getY());
            telemetry.addData("Pinpoint heading", pinpoint.getHeading());
            telemetry.addData("FC field heading", fieldHeading);
            telemetry.addData("Heading offset", headingOffset);
            telemetry.addData("Alliance forward", allianceForward);
            telemetry.addData("Bad reads / jumps",
                    pinpoint.getBadReads() + " / " + pinpoint.getJumpCount());
            telemetry.addData("Square",
                    "Calibrate ONLY when intake faces opposite alliance wall");
            telemetry.update();
        }
        hwMap.getFrontLeftMotor().setPower(0);
        hwMap.getFrontRightMotor().setPower(0);
        hwMap.getBackLeftMotor().setPower(0);
        hwMap.getBackRightMotor().setPower(0);
    }
}
