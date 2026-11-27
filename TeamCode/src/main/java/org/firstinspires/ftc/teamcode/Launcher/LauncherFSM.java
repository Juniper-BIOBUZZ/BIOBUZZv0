package org.firstinspires.ftc.teamcode.Launcher;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Core.HWMap;
import org.firstinspires.ftc.teamcode.Core.Logger;
import org.firstinspires.ftc.teamcode.Core.Pinpoint;
import org.firstinspires.ftc.teamcode.Core.RobotSettings;

@Config
public class LauncherFSM {

    public enum State {
        OFF,
        AIMING,
        READY,
        FIRING,
        RECOVERING,
        FAULTED
    }

    public static double SHOT_DROP_RPM = 100;
    public static double FIRE_TIMEOUT = 1.0;
    public static double MIN_RECOVER_TIME = 0.15;

    public static double PITCH_COMPENSATION_ERROR_THRESHOLD = 40;

    public final TurretFSM turretFSM;
    public final PitchFSM pitchFSM;
    public final FlywheelFSM flywheelFSM;
    public final PositionFSM positionFSM;

    private final Logger logger;

    private State state = State.OFF;

    private boolean enabled = false;
    private boolean manual = false;

    private double manualRPM = 0;
    private double manualHood = 0;

    private final ElapsedTime stateTimer = new ElapsedTime();

    private double lowestRPMThisShot = 0;
    private double lastShotDip = 0;

    private double baseHoodTarget = 0;
    private double compensatedHoodTarget = 0;
    private double pitchReductionFactor = 0;
    private double flywheelError = 0;

    public LauncherFSM(
            HWMap hwMap,
            Pinpoint pinpoint,
            RobotSettings settings,
            Logger logger
    ) {
        this.logger = logger;

        turretFSM = new TurretFSM(hwMap, logger);
        pitchFSM = new PitchFSM(hwMap, logger);
        flywheelFSM = new FlywheelFSM(hwMap, logger);
        positionFSM = new PositionFSM(pinpoint, settings, logger);
    }

    public void updateState() {
        positionFSM.updateState();

        double rpm = manual
                ? manualRPM
                : positionFSM.getTargetRPM();

        baseHoodTarget = manual
                ? manualHood
                : positionFSM.getHoodTarget();

        boolean trackingValid = positionFSM.isTrackingValid();

        if (trackingValid) {
            turretFSM.setTargetAngle(positionFSM.getTurretTarget());
        }

        turretFSM.updateState(trackingValid);

        boolean turretFaulted = turretFSM.FAULTED();

        flywheelFSM.setTargetRPM(
                !turretFaulted && trackingValid
                        ? rpm
                        : 0
        );

        flywheelFSM.updateState();

        flywheelError = rpm - flywheelFSM.getCurrentRPM();
        pitchReductionFactor = manual
                ? 0
                : positionFSM.getPitchReductionFactor();

        compensatedHoodTarget = baseHoodTarget;

        if (!manual
                && trackingValid
                && flywheelFSM.getCurrentRPM() < rpm
                && flywheelError > PITCH_COMPENSATION_ERROR_THRESHOLD) {

            compensatedHoodTarget =
                    baseHoodTarget
                            - flywheelError * pitchReductionFactor;
        }

        if (trackingValid) {
            pitchFSM.setTargetPosition(compensatedHoodTarget);
        }

        pitchFSM.updateState();

        boolean faulted =
                turretFSM.FAULTED()
                        || pitchFSM.FAULTED();

        if (faulted) {
            flywheelFSM.setTargetRPM(0);
            setState(State.FAULTED);
            return;
        }

        if (!enabled) {
            setState(State.OFF);
            return;
        }

        if (!trackingValid) {
            setState(State.AIMING);
            return;
        }

        boolean aimed =
                turretFSM.ALIGNED()
                        && pitchFSM.ALIGNED()
                        && flywheelFSM.AT_SPEED()
                        && !positionFSM.OUT_OF_RANGE();

        switch (state) {
            case OFF:
            case FAULTED:
                setState(State.AIMING);
                break;

            case AIMING:
                if (aimed) {
                    setState(State.READY);
                }
                break;

            case READY:
                if (!aimed) {
                    setState(State.AIMING);
                }
                break;

            case FIRING:
                lowestRPMThisShot = Math.min(
                        lowestRPMThisShot,
                        flywheelFSM.getCurrentRPM()
                );

                boolean ballWentThrough =
                        flywheelFSM.getCurrentRPM()
                                < flywheelFSM.getTargetRPM() - SHOT_DROP_RPM;

                if (ballWentThrough
                        || stateTimer.seconds() > FIRE_TIMEOUT) {

                    lastShotDip =
                            flywheelFSM.getTargetRPM()
                                    - lowestRPMThisShot;

                    setState(State.RECOVERING);
                }
                break;

            case RECOVERING:
                lowestRPMThisShot = Math.min(
                        lowestRPMThisShot,
                        flywheelFSM.getCurrentRPM()
                );

                lastShotDip = Math.max(
                        lastShotDip,
                        flywheelFSM.getTargetRPM()
                                - lowestRPMThisShot
                );

                if (stateTimer.seconds() > MIN_RECOVER_TIME && aimed) {
                    setState(State.READY);
                }
                break;
        }
    }

    private void setState(State newState) {
        if (state != newState) {
            state = newState;
            stateTimer.reset();
        }
    }

    public boolean isReady() {
        return state == State.READY
                && positionFSM.isTrackingValid();
    }

    public void shotStarted() {
        if (isReady()) {
            lowestRPMThisShot = flywheelFSM.getCurrentRPM();
            setState(State.FIRING);
        }
    }

    public void setEnabled(boolean enabled) {
        this.enabled = enabled;
    }

    public void toggleEnabled() {
        enabled = !enabled;
    }

    public boolean isEnabled() {
        return enabled;
    }

    public void setManual(
            boolean manual,
            double rpm,
            double hood
    ) {
        this.manual = manual;
        manualRPM = rpm;
        manualHood = hood;
    }

    public void switchCell() {
        positionFSM.switchCell();
    }

    public State getState() {
        return state;
    }

    public void log() {
        logger.log(
                "<font color='yellow'>----------LAUNCHER----------</font>",
                "",
                Logger.LogLevels.PRODUCTION
        );

        logger.log(
                "Launcher state",
                state,
                Logger.LogLevels.PRODUCTION
        );

        logger.log(
                "<b>Ready to fire</b>",
                isReady()
                        ? "<font color='green'><b>YES</b></font>"
                        : "no",
                Logger.LogLevels.PRODUCTION
        );

        logger.log(
                "Launcher mode",
                manual ? "manual" : "auto",
                Logger.LogLevels.DEBUG
        );

        logger.log(
                "Last shot rpm dip",
                String.format("%.0f", lastShotDip),
                Logger.LogLevels.DEBUG
        );

        logger.log(
                "Flywheel error",
                String.format("%.0f", flywheelError),
                Logger.LogLevels.DEBUG
        );

        logger.log(
                "Base / compensated pitch",
                String.format(
                        "%.3f / %.3f",
                        baseHoodTarget,
                        compensatedHoodTarget
                ),
                Logger.LogLevels.DEBUG
        );

        logger.log(
                "Pitch reduction factor",
                String.format("%.5f", pitchReductionFactor),
                Logger.LogLevels.DEBUG
        );

        positionFSM.log();
        turretFSM.log();
        pitchFSM.log();
        flywheelFSM.log();
    }
}
