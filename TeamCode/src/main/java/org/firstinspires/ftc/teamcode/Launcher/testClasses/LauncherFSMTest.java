
package org.firstinspires.ftc.teamcode.Launcher.testClasses;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Core.Drivetrain;
import org.firstinspires.ftc.teamcode.Core.HWMap;
import org.firstinspires.ftc.teamcode.Core.Logger;
import org.firstinspires.ftc.teamcode.Core.MechanumHardwareMap;
import org.firstinspires.ftc.teamcode.Core.Pinpoint;
import org.firstinspires.ftc.teamcode.Core.RobotSettings;
import org.firstinspires.ftc.teamcode.Launcher.LauncherFSM;

/*
 * Launcher FSM test:
 *   run Robot Settings first and pick the alliance
 *   put the robot touching the starting wall, center 24 in from the center line, turret facing back
 *   push the hood all the way down before START; it zeros on START
 *   wait for Start pose set = YES and Pinpoint status = READY
 *   drive field centric using Pinpoint, forward goes toward the opposite alliance wall
 *   Options recalibrates FC ONLY when intake faces the opposite alliance wall
 *   Cross toggles flywheel
 *   Square switches hive cell
 *   Triangle toggles manual RPM and hood adjustment
 *   Dpad up/down changes RPM +/-25 in manual mode
 *   Dpad left/right changes hood +/-1 in manual mode
 *   R1 marks a shot
 *   Share switches DEBUG/PRODUCTION telemetry
 */

@TeleOp(group = "Launcher", name = "Launcher FSM Test")
public class LauncherFSMTest extends LinearOpMode {

    private boolean manual = false;
    private double manualRPM = 1575, manualHood = 15;
    private boolean lastA, lastX, lastY, lastRB, lastUp, lastDown, lastLeft, lastRight, lastShare, lastOptions;
    private double fieldHeading = 0, headingOffset = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        Logger logger = new Logger(telemetry);
        HWMap hwMap = new HWMap(hardwareMap);
        RobotSettings settings = RobotSettings.load();
        MechanumHardwareMap mechanumHardwareMap = new MechanumHardwareMap(hardwareMap);

        Pinpoint pinpoint = new Pinpoint(mechanumHardwareMap);
        Pose2D start = RobotSettings.StartPos.forAlliance(settings.alliance).getPose2D();
        pinpoint.setPosition(start);

        LauncherFSM launcher = new LauncherFSM(hwMap, pinpoint, settings, logger);

        double allianceForward = settings.alliance == RobotSettings.Alliance.RED ? -90 : 90;
        Drivetrain drivetrain = new Drivetrain(telemetry, gamepad1, mechanumHardwareMap,
                () -> fieldHeading, settings.alliance);
        drivetrain.setFieldCentric(true);

        while (opModeInInit()) {
            pinpoint.update();

            logger.log("<b><font color='orange'>Turret facing back and hood all the way down</font></b>", "", Logger.LogLevels.PRODUCTION);
            logger.log("Alliance", settings.alliance, Logger.LogLevels.PRODUCTION);
            logger.log("Start position", RobotSettings.StartPos.forAlliance(settings.alliance), Logger.LogLevels.PRODUCTION);
            logger.log("Start pose set", pinpoint.positionSet() ? "<font color='green'>YES</font>" : "<font color='red'>not yet</font>", Logger.LogLevels.PRODUCTION);
            logger.log("Pinpoint status", pinpoint.getStatus() + ", bad reads " + pinpoint.getBadReads(), Logger.LogLevels.PRODUCTION);
            logger.log("Should be x / y / heading", String.format("%.1f / %.1f / %.1f",
                    start.getX(DistanceUnit.INCH),
                    start.getY(DistanceUnit.INCH),
                    start.getHeading(AngleUnit.DEGREES)), Logger.LogLevels.PRODUCTION);
            logger.log("Pinpoint x / y / heading", String.format("%.1f / %.1f / %.1f",
                    pinpoint.getX(), pinpoint.getY(), pinpoint.getHeading()), Logger.LogLevels.PRODUCTION);
            logger.print();
            idle();
        }

        if (isStopRequested()) return;
        if (!pinpoint.positionSet() || !"READY".equals(pinpoint.getStatus())) {
            telemetry.addLine("Start pose not verified. STOP and INIT from the starting wall.");
            telemetry.update();
            return;
        }

        while (opModeIsActive()) {
            logger.updateLoggingLevel(gamepad1.back && !lastShare);

            if (gamepad1.a && !lastA) launcher.toggleEnabled();
            if (gamepad1.x && !lastX) launcher.switchCell();
            if (gamepad1.y && !lastY) manual = !manual;

            if (manual) {
                if (gamepad1.dpad_up && !lastUp) manualRPM += 25;
                if (gamepad1.dpad_down && !lastDown) manualRPM = Math.max(0, manualRPM - 25);
                if (gamepad1.dpad_right && !lastRight) manualHood += 1;
                if (gamepad1.dpad_left && !lastLeft) manualHood -= 1;
            }

            lastShare = gamepad1.back;
            lastA = gamepad1.a;
            lastX = gamepad1.x;
            lastY = gamepad1.y;
            lastUp = gamepad1.dpad_up;
            lastDown = gamepad1.dpad_down;
            lastLeft = gamepad1.dpad_left;
            lastRight = gamepad1.dpad_right;

            launcher.setManual(manual, manualRPM, manualHood);
            launcher.updateState();

            if (gamepad1.right_bumper && !lastRB) launcher.shotStarted();
            lastRB = gamepad1.right_bumper;

            boolean valid = pinpoint.positionSet()
                    && "READY".equals(pinpoint.getStatus())
                    && Double.isFinite(pinpoint.getHeading())
                    && Double.isFinite(pinpoint.getX())
                    && Double.isFinite(pinpoint.getY());

            if (valid) {
                fieldHeading = AngleUnit.normalizeDegrees(pinpoint.getHeading() + headingOffset);

                if (gamepad1.start && !lastOptions) {
                    headingOffset = AngleUnit.normalizeDegrees(allianceForward - pinpoint.getHeading());
                    fieldHeading = allianceForward;
                }
            }
            lastOptions = gamepad1.start;

            if (valid) {
                drivetrain.update();
            } else {
                mechanumHardwareMap.getFrontLeftMotor().setPower(0);
                mechanumHardwareMap.getFrontRightMotor().setPower(0);
                mechanumHardwareMap.getBackLeftMotor().setPower(0);
                mechanumHardwareMap.getBackRightMotor().setPower(0);
            }

            logger.log("Flywheel on (Cross)", launcher.isEnabled(), Logger.LogLevels.PRODUCTION);
            logger.log("Drive mode", "FIELD CENTRIC (Pinpoint)", Logger.LogLevels.PRODUCTION);
            logger.log("Pinpoint valid for FC", valid, Logger.LogLevels.PRODUCTION);
            logger.log("FC field heading", String.format("%.1f", fieldHeading), Logger.LogLevels.DEBUG);
            logger.log("Pinpoint heading", String.format("%.1f", pinpoint.getHeading()), Logger.LogLevels.DEBUG);
            logger.log("Heading offset", String.format("%.1f", headingOffset), Logger.LogLevels.DEBUG);
            logger.log("Pinpoint status", pinpoint.getStatus(), Logger.LogLevels.DEBUG);
            logger.log("Pinpoint bad reads / jumps", pinpoint.getBadReads() + " / " + pinpoint.getJumpCount(), Logger.LogLevels.DEBUG);

            if (manual) {
                logger.log("Manual rpm (dpad up/down)", manualRPM, Logger.LogLevels.PRODUCTION);
                logger.log("Manual hood (dpad L/R)", manualHood, Logger.LogLevels.PRODUCTION);
            }

            launcher.log();
            logger.print();
        }

        mechanumHardwareMap.getFrontLeftMotor().setPower(0);
        mechanumHardwareMap.getFrontRightMotor().setPower(0);
        mechanumHardwareMap.getBackLeftMotor().setPower(0);
        mechanumHardwareMap.getBackRightMotor().setPower(0);
    }
}
