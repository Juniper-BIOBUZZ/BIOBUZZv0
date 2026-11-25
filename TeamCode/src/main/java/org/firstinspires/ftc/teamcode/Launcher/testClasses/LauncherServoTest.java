package org.firstinspires.ftc.teamcode.Launcher.testClasses;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Core.AxonServoModeWrapper;
import org.firstinspires.ftc.teamcode.Core.HWMap;

@Config
public class LauncherServoTest {

    private final AxonServoModeWrapper pitchServo;

    public static double targetPosition = 0.30;
    public static boolean INVERT_POSITION = false;
    public static boolean INVERT_ENCODER = false;
    public static double ENCODER_OFFSET = 0.0;

    private final Telemetry telemetry;

    public LauncherServoTest(Telemetry telemetry, HWMap hwMap) {
        this.telemetry = new MultipleTelemetry(
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
    }

    public void updatePID() {
        pitchServo.setPosition(targetPosition);
        pitchServo.readPos();

        telemetry.addData("Pitch target position", "%.3f", targetPosition);
        telemetry.addData("Pitch commanded position", "%.3f", pitchServo.getPosition());
        telemetry.addData("Pitch encoder angle", "%.1f", pitchServo.getServoAngle());
        telemetry.addData("Pitch encoder voltage", "%.3f", pitchServo.readAndGetVoltage());
    }
}