package org.firstinspires.ftc.teamcode.Launcher;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.Core.AxonServoModeWrapper;
import org.firstinspires.ftc.teamcode.Core.HWMap;
import org.firstinspires.ftc.teamcode.Core.Logger;

@Config
public class PitchFSM {

    public enum State {
        ALIGNING,
        ALIGNED,
        FAULTED
    }

    public static boolean INVERT_POSITION = false;
    public static boolean INVERT_ENCODER = false;
    public static double ENCODER_OFFSET = 0.0;

    private final AxonServoModeWrapper pitchServo;
    private final Logger logger;

    private State state = State.ALIGNED;
    private double targetPosition = 0.0;

    public PitchFSM(HWMap hwMap, Logger logger) {
        this.logger = logger;

        pitchServo = new AxonServoModeWrapper(
                hwMap.getPitchServo(),
                hwMap.getPitchEncoder(),
                INVERT_POSITION,
                INVERT_ENCODER,
                ENCODER_OFFSET
        );
    }

    public void setTargetPosition(double position) {
        targetPosition = position;
    }

    public void updateState() {
        pitchServo.setPosition(targetPosition);
        pitchServo.readPos();

        state = State.ALIGNED;
    }

    public boolean ALIGNED() {
        return state == State.ALIGNED;
    }

    public boolean FAULTED() {
        return state == State.FAULTED;
    }

    public String getFaultReason() {
        return "none";
    }

    public double getTargetPosition() {
        return targetPosition;
    }

    public double getCurrentPosition() {
        return pitchServo.getPosition();
    }

    public double getEncoderAngle() {
        return pitchServo.getServoAngle();
    }

    public void log() {
        logger.log(
                "<font color='yellow'>----------PITCH----------</font>",
                "",
                Logger.LogLevels.PRODUCTION
        );

        logger.log(
                "Pitch state",
                state,
                Logger.LogLevels.PRODUCTION
        );

        logger.log(
                "Pitch target position",
                String.format("%.3f", targetPosition),
                Logger.LogLevels.PRODUCTION
        );

        logger.log(
                "Pitch commanded position",
                String.format("%.3f", pitchServo.getPosition()),
                Logger.LogLevels.PRODUCTION
        );

        logger.log(
                "Pitch encoder angle",
                String.format("%.1f", pitchServo.getServoAngle()),
                Logger.LogLevels.PRODUCTION
        );

        logger.log(
                "Pitch encoder voltage",
                String.format("%.3f", pitchServo.readAndGetVoltage()),
                Logger.LogLevels.DEBUG
        );
    }
}