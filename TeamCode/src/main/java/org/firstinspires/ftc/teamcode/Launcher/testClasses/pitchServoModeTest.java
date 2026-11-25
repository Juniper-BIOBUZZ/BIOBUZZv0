package org.firstinspires.ftc.teamcode.Launcher.testClasses;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.Core.AxonServoModeWrapper;

@Config
@TeleOp(group = "Launcher", name = "Pitch Servo Mode Test")
public class pitchServoModeTest extends LinearOpMode {

    private AxonServoModeWrapper pitchServo;
    public static double targetPosition = 0.50;
    public static boolean INVERT_POSITION = false;
    public static boolean INVERT_ENCODER = false;
    public static double ENCODER_OFFSET = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Servo servo = hardwareMap.get(Servo.class, "PS");
        AnalogInput encoder = hardwareMap.get(AnalogInput.class, "PE");
        pitchServo = new AxonServoModeWrapper(
                servo,
                encoder,
                INVERT_POSITION,
                INVERT_ENCODER,
                ENCODER_OFFSET
        );
        waitForStart();
        while (opModeIsActive()) {
            pitchServo.setPosition(targetPosition);
            pitchServo.readPos();
            telemetry.addData("Target position", "%.3f", targetPosition);
            telemetry.addData("Commanded position", "%.3f", pitchServo.getPosition());
            telemetry.addData("Encoder voltage", "%.3f", pitchServo.readAndGetVoltage());
            telemetry.addData("Encoder angle", "%.1f", pitchServo.getServoAngle());
            telemetry.update();
        }
    }
}