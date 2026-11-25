package org.firstinspires.ftc.teamcode.Launcher.testClasses;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Core.AxonServoModeWrapper;
import org.firstinspires.ftc.teamcode.Core.HWMap;

@Config
@TeleOp(group = "Launcher", name = "Pitch Servo Position Test")
public class PitchPIDTest extends LinearOpMode {

    private AxonServoModeWrapper pitchServo;

    public static double targetPosition = 0.30;
    public static double MIN_POSITION = 0.0;
    public static double MAX_POSITION = 1.0;

    public static boolean INVERT_POSITION = false;
    public static boolean INVERT_ENCODER = false;
    public static double ENCODER_OFFSET = 0.0;

    @Override
    public void runOpMode() throws InterruptedException {

        HWMap hwMap = new HWMap(hardwareMap);

        telemetry = new MultipleTelemetry(
                telemetry,
                FtcDashboard.getInstance().getTelemetry()
        );

        pitchServo = new AxonServoModeWrapper(
                hwMap.getPitchServo(),
                hwMap.getPitchEncoder(),
                INVERT_POSITION,
                INVERT_ENCODER,
                ENCODER_OFFSET
        );

        waitForStart();

        while (opModeIsActive()) {

            updatePitch();

            telemetry.addData("Pitch target position", "%.3f", targetPosition);
            telemetry.addData("Pitch commanded position", "%.3f", pitchServo.getPosition());
            telemetry.addData("Pitch encoder angle", "%.1f", pitchServo.getServoAngle());
            telemetry.addData("Pitch encoder voltage", "%.3f", pitchServo.readAndGetVoltage());
            telemetry.update();
        }
    }

    public void updatePitch() {

        targetPosition = Range.clip(
                targetPosition,
                MIN_POSITION,
                MAX_POSITION
        );

        pitchServo.setPosition(targetPosition);
        pitchServo.readPos();
    }
}