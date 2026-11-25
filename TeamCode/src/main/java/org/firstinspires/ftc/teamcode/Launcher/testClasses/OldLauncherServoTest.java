package org.firstinspires.ftc.teamcode.Launcher.testClasses;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Core.AxonServoModeWrapper;
import org.firstinspires.ftc.teamcode.Core.HWMap;

@Config
@TeleOp(group = "Launcher", name = "Pitch Bottom/Top Test")
public class OldLauncherServoTest extends LinearOpMode {

    public static double MIN_POSITION = 0.0;
    public static double MAX_POSITION = 1.0;
    public static double STEP = 0.02;

    public static boolean INVERT_POSITION = false;
    public static boolean INVERT_ENCODER = false;
    public static double ENCODER_OFFSET = 0.0;

    public static double targetPosition = 0.0;

    private AxonServoModeWrapper pitchServo;

    private boolean lastX;
    private boolean lastY;
    private boolean lastUp;
    private boolean lastDown;

    @Override
    public void runOpMode() throws InterruptedException {

        HWMap hwMap = new HWMap(hardwareMap);

        pitchServo = new AxonServoModeWrapper(
                hwMap.getPitchServo(),
                hwMap.getPitchEncoder(),
                INVERT_POSITION,
                INVERT_ENCODER,
                ENCODER_OFFSET
        );

        targetPosition = MIN_POSITION;

        telemetry.addLine("Axon pitch servo mode test");
        telemetry.addLine("D-pad up/down changes target");
        telemetry.addLine("Triangle = top");
        telemetry.addLine("Square = bottom");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            readButtons();

            targetPosition = Range.clip(
                    targetPosition,
                    MIN_POSITION,
                    MAX_POSITION
            );

            pitchServo.setPosition(targetPosition);
            pitchServo.readPos();

            telemetry.addData("Target position", "%.3f", targetPosition);
            telemetry.addData("Commanded position", "%.3f", pitchServo.getPosition());
            telemetry.addData("Encoder angle", "%.1f", pitchServo.getServoAngle());
            telemetry.addData("Encoder voltage", "%.3f", pitchServo.readAndGetVoltage());
            telemetry.update();
        }
    }

    private void readButtons() {

        if (gamepad1.dpad_up && !lastUp) {
            targetPosition += STEP;
        }

        if (gamepad1.dpad_down && !lastDown) {
            targetPosition -= STEP;
        }

        if (gamepad1.y && !lastY) {
            targetPosition = MAX_POSITION;
        }

        if (gamepad1.x && !lastX) {
            targetPosition = MIN_POSITION;
        }

        lastUp = gamepad1.dpad_up;
        lastDown = gamepad1.dpad_down;
        lastY = gamepad1.y;
        lastX = gamepad1.x;
    }
}