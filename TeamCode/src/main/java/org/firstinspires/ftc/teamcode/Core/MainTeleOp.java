package org.firstinspires.ftc.teamcode.Core;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Intake.IntakeFSM;
import org.firstinspires.ftc.teamcode.Launcher.LauncherFSM;
import org.firstinspires.ftc.teamcode.Transfer.TransferFSM;
import org.firstinspires.ftc.teamcode.Transfer.TransferMotorFSM;
import org.firstinspires.ftc.teamcode.Transfer.GateFSM;

@TeleOp(group = "Launcher", name = "MainTeleOp")
public class MainTeleOp extends LinearOpMode {

    private static final double BELT_TARGET_RPM = 700;

    @Override
    public void runOpMode() throws InterruptedException {
        Logger logger = new Logger(telemetry);
        HWMap hwMap = new HWMap(hardwareMap);
        MechanumHardwareMap driveMap = new MechanumHardwareMap(hardwareMap);
        NewIntakeTransferHWmap intakeTransferMap =
                new NewIntakeTransferHWmap(hardwareMap);

        RobotSettings settings = RobotSettings.load();

        Pose2D start =
                RobotSettings.StartPos
                        .forAlliance(settings.alliance)
                        .getPose2D();

        Pinpoint pinpoint = new Pinpoint(driveMap);
        pinpoint.setPosition(start);

        LauncherFSM launcher =
                new LauncherFSM(
                        hwMap,
                        pinpoint,
                        settings,
                        logger
                );

        launcher.setEnabled(true);

        TransferMotorFSM.targetVelocity = 0;
        GateFSM.targetPosition = GateFSM.positionDown;

        TransferFSM transferFSM =
                new TransferFSM(
                        intakeTransferMap,
                        telemetry
                );

        IntakeFSM intakeFSM =
                new IntakeFSM(
                        intakeTransferMap,
                        telemetry,
                        transferFSM
                );

        double allianceForward =
                settings.alliance == RobotSettings.Alliance.RED
                        ? -90
                        : 90;

        final double[] fieldHeading = {
                start.getHeading(AngleUnit.DEGREES)
        };

        double headingOffset = 0;

        Drivetrain drivetrain =
                new Drivetrain(
                        telemetry,
                        gamepad1,
                        driveMap,
                        () -> fieldHeading[0],
                        settings.alliance
                );

        drivetrain.setFieldCentric(true);

        boolean manual = false;
        boolean beltEnabled = true;

        double manualRPM = 1500;
        double manualHood = 10;

        boolean lastA = false;
        boolean lastX = false;
        boolean lastY = false;
        boolean lastB = false;
        boolean lastLB = false;
        boolean lastRB = false;
        boolean lastRight = false;
        boolean lastLeft = false;
        boolean lastUp = false;
        boolean lastBack = false;
        boolean lastOptions = false;
        boolean lastTrigger = false;
        boolean lastL2 = false;

        while (opModeInInit()) {
            pinpoint.update();

            logger.log(
                    "<b><font color='orange'>Turret at zero; hood fully down BEFORE START</font></b>",
                    "",
                    Logger.LogLevels.PRODUCTION
            );

            logger.log(
                    "Alliance",
                    settings.alliance,
                    Logger.LogLevels.PRODUCTION
            );

            logger.log(
                    "Start pose set",
                    pinpoint.positionSet(),
                    Logger.LogLevels.PRODUCTION
            );

            logger.log(
                    "Pinpoint status",
                    pinpoint.getStatus(),
                    Logger.LogLevels.PRODUCTION
            );

            logger.log(
                    "Expected X / Y / heading",
                    String.format(
                            "%.1f / %.1f / %.1f",
                            start.getX(DistanceUnit.INCH),
                            start.getY(DistanceUnit.INCH),
                            start.getHeading(AngleUnit.DEGREES)
                    ),
                    Logger.LogLevels.PRODUCTION
            );

            logger.log(
                    "Pinpoint X / Y / heading",
                    String.format(
                            "%.1f / %.1f / %.1f",
                            pinpoint.getX(),
                            pinpoint.getY(),
                            pinpoint.getHeading()
                    ),
                    Logger.LogLevels.PRODUCTION
            );

            logger.print();
            idle();
        }

        if (isStopRequested()) {
            return;
        }

        try {
            while (opModeIsActive()) {
                boolean a = gamepad1.a;
                boolean x = gamepad1.x;
                boolean y = gamepad1.y;
                boolean b = gamepad1.b;

                boolean lb = gamepad1.left_bumper;
                boolean rb = gamepad1.right_bumper;

                boolean right = gamepad1.dpad_right;
                boolean left = gamepad1.dpad_left;
                boolean up = gamepad1.dpad_up;

                boolean back = gamepad1.back;
                boolean options = gamepad1.start;

                boolean trigger =
                        gamepad1.right_trigger > 0.5;

                boolean l2 =
                        gamepad1.left_trigger > 0.5;

                logger.updateLoggingLevel(
                        back && !lastBack
                );

                if (a && !lastA) {
                    launcher.toggleEnabled();
                }

                if (x && !lastX) {
                    launcher.switchCell();
                }

                if (y && !lastY) {
                    manual = !manual;
                }

                if (l2 && !lastL2) {
                    intakeFSM.forceStop();
                    beltEnabled = false;
                    TransferMotorFSM.targetVelocity = 0;
                }

                if (manual) {
                    if (up && !lastUp) {
                        manualRPM += 25;
                    }
                } else {
                    if (up) {
                        drivetrain.setFieldCentric(true);
                    }

                    if (right && !lastRight) {
                        beltEnabled = !beltEnabled;
                    }

                    if (left && !lastLeft) {
                        beltEnabled = false;
                    }
                }

                launcher.setManual(
                        manual,
                        manualRPM,
                        manualHood
                );

                launcher.updateState();

                if (rb && !lastRB) {
                    launcher.shotStarted();
                }

                intakeFSM.updateState(
                        b && !lastB,
                        lb && !lastLB
                );

                TransferMotorFSM.targetVelocity =
                        beltEnabled
                                ? BELT_TARGET_RPM
                                : 0;

                transferFSM.updateState(
                        trigger != lastTrigger,
                        false
                );

                boolean poseValid =
                        pinpoint.positionSet()
                                && "READY".equals(
                                pinpoint.getStatus()
                        )
                                && Double.isFinite(
                                pinpoint.getHeading()
                        )
                                && Double.isFinite(
                                pinpoint.getX()
                        )
                                && Double.isFinite(
                                pinpoint.getY()
                        );

                if (poseValid) {
                    fieldHeading[0] =
                            AngleUnit.normalizeDegrees(
                                    pinpoint.getHeading()
                                            + headingOffset
                            );

                    if (options && !lastOptions) {
                        headingOffset =
                                AngleUnit.normalizeDegrees(
                                        allianceForward
                                                - pinpoint.getHeading()
                                );

                        fieldHeading[0] =
                                allianceForward;
                    }
                }

                if (poseValid
                        || !drivetrain.isFieldCentric()) {

                    drivetrain.update();

                } else {
                    driveMap.getFrontLeftMotor()
                            .setPower(0);

                    driveMap.getFrontRightMotor()
                            .setPower(0);

                    driveMap.getBackLeftMotor()
                            .setPower(0);

                    driveMap.getBackRightMotor()
                            .setPower(0);
                }

                logger.log(
                        "<font color='yellow'>----------INTAKE / TRANSFER----------</font>",
                        "",
                        Logger.LogLevels.PRODUCTION
                );

                logger.log(
                        "Intake controls",
                        "L1: stop/start; Circle: intake/eject",
                        Logger.LogLevels.PRODUCTION
                );

                logger.log(
                        "Combined stop",
                        "L2: stop intake + transfer",
                        Logger.LogLevels.PRODUCTION
                );

                logger.log(
                        "Transfer belt",
                        beltEnabled
                                ? "RUNNING ("
                                + BELT_TARGET_RPM
                                + " RPM)"
                                : "STOPPED",
                        Logger.LogLevels.PRODUCTION
                );

                logger.log(
                        "Gate (R2)",
                        trigger
                                ? "UP requested"
                                : "DOWN requested",
                        Logger.LogLevels.PRODUCTION
                );

                logger.log(
                        "Pinpoint drive valid",
                        poseValid,
                        Logger.LogLevels.PRODUCTION
                );

                logger.log(
                        "FC field heading",
                        String.format(
                                "%.1f",
                                fieldHeading[0]
                        ),
                        Logger.LogLevels.DEBUG
                );

                logger.log(
                        "Pinpoint X / Y / heading",
                        String.format(
                                "%.1f / %.1f / %.1f",
                                pinpoint.getX(),
                                pinpoint.getY(),
                                pinpoint.getHeading()
                        ),
                        Logger.LogLevels.DEBUG
                );

                logger.log(
                        "Flywheel enabled",
                        launcher.isEnabled(),
                        Logger.LogLevels.PRODUCTION
                );

                if (manual) {
                    logger.log(
                            "Manual RPM",
                            manualRPM,
                            Logger.LogLevels.PRODUCTION
                    );

                    logger.log(
                            "Manual hood",
                            manualHood,
                            Logger.LogLevels.PRODUCTION
                    );
                }

                launcher.log();
                logger.print();

                lastA = a;
                lastX = x;
                lastY = y;
                lastB = b;
                lastLB = lb;
                lastRB = rb;
                lastRight = right;
                lastLeft = left;
                lastUp = up;
                lastBack = back;
                lastOptions = options;
                lastTrigger = trigger;
                lastL2 = l2;
            }

        } finally {
            driveMap.getFrontLeftMotor()
                    .setPower(0);

            driveMap.getFrontRightMotor()
                    .setPower(0);

            driveMap.getBackLeftMotor()
                    .setPower(0);

            driveMap.getBackRightMotor()
                    .setPower(0);

            TransferMotorFSM.targetVelocity = 0;

            intakeTransferMap.getIntakeMotor()
                    .stopMotor();

            intakeTransferMap.getTransferMotor()
                    .stopMotor();

            hwMap.getFlywheelMotor()
                    .stopMotor();

            hwMap.getTurretMotor()
                    .stopMotor();

            hwMap.getPitchServo()
                    .setPosition(0);

            GateFSM.targetPosition =
                    GateFSM.positionDown;

            intakeTransferMap.getTransferServo()
                    .setPosition(
                            GateFSM.positionDown
                    );
        }
    }
}
