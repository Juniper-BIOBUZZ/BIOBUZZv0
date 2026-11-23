package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Core.Drivetrain;
import org.firstinspires.ftc.teamcode.Core.MechanumHardwareMap;
import org.firstinspires.ftc.teamcode.Core.RobotSettings;

@TeleOp(group = "Drivetrain", name = "Drivetrain Test")
public class DrivetrainTest extends LinearOpMode {

    private IMU imu;
    private double headingOffset;
    private double fieldHeading;
    private double rawYaw;
    private boolean headingCalibrated;

    private boolean updateHeading(double startHeading) {
        rawYaw = imu.getRobotYawPitchRollAngles()
                .getYaw(AngleUnit.DEGREES);

        if (!Double.isFinite(rawYaw)) return false;

        if (!headingCalibrated) {
            headingOffset = startHeading - rawYaw;
            headingCalibrated = true;
        }

        fieldHeading = AngleUnit.normalizeDegrees(
                rawYaw + headingOffset);

        return true;
    }

    @Override
    public void runOpMode() {

        RobotSettings settings = RobotSettings.load();

        MechanumHardwareMap hwMap =
                new MechanumHardwareMap(hardwareMap);

        imu = hardwareMap.get(IMU.class, "imu");

        if (!imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                        RevHubOrientationOnRobot.UsbFacingDirection.LEFT)))) {

            telemetry.addData("IMU", "Initialization failed");
            telemetry.update();
            return;
        }

        double startHeading = settings.startPosState
                .getPose2D().getHeading(AngleUnit.DEGREES);

        double allianceForward =
                settings.alliance == RobotSettings.Alliance.RED
                        ? -90 : 90;

        headingCalibrated = false;
        updateHeading(startHeading);

        Drivetrain drivetrain = new Drivetrain(
                telemetry,
                gamepad1,
                hwMap,
                () -> fieldHeading,
                settings.alliance);

        drivetrain.setFieldCentric(true);

        while (opModeInInit()) {

            boolean valid = updateHeading(startHeading);

            telemetry.addData("Alliance", settings.alliance);
            telemetry.addData("Mode", "FIELD CENTRIC");
            telemetry.addData("IMU valid", valid);
            telemetry.addData("Raw yaw", rawYaw);
            telemetry.addData("Field heading", fieldHeading);
            telemetry.addData("Alliance forward", allianceForward);

            telemetry.update();
            idle();
        }

        boolean lastSquare = gamepad1.x;

        while (opModeIsActive()) {

            boolean valid = updateHeading(startHeading);

            if (gamepad1.dpad_up) {
                drivetrain.setFieldCentric(true);
            } else if (gamepad1.dpad_down) {
                drivetrain.setFieldCentric(false);
            }

            if (valid && gamepad1.x && !lastSquare) {
                headingOffset = allianceForward - rawYaw;
                fieldHeading = allianceForward;
            }

            lastSquare = gamepad1.x;

            if (valid || !drivetrain.isFieldCentric()) {
                drivetrain.update();
            } else {
                hwMap.getFrontLeftMotor().setPower(0);
                hwMap.getFrontRightMotor().setPower(0);
                hwMap.getBackLeftMotor().setPower(0);
                hwMap.getBackRightMotor().setPower(0);
            }

            telemetry.addData("Mode (Dpad Up FC / Down RC)",
                    drivetrain.isFieldCentric()
                            ? "FIELD CENTRIC" : "ROBOT CENTRIC");

            telemetry.addData("IMU valid", valid);
            telemetry.addData("Raw yaw", rawYaw);
            telemetry.addData("Field heading", fieldHeading);

            telemetry.addData("Heading difference",
                    AngleUnit.normalizeDegrees(
                            allianceForward - fieldHeading));

            telemetry.addData("Square",
                    "Calibrate facing opposing alliance");

            telemetry.update();
        }

        hwMap.getFrontLeftMotor().setPower(0);
        hwMap.getFrontRightMotor().setPower(0);
        hwMap.getBackLeftMotor().setPower(0);
        hwMap.getBackRightMotor().setPower(0);
    }
}