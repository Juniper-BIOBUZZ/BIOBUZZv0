package org.firstinspires.ftc.teamcode.Launcher;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Core.HWMap;
import org.firstinspires.ftc.teamcode.Core.Logger;
import org.firstinspires.ftc.teamcode.Core.MotorWrapper;

@Config
public class FlywheelFSM {

    public enum State {
        STOPPED,
        SPINNING_UP,
        AT_SPEED
    }

    public static double vP = 4, vI = 5, vD = 0;
    public static double kS = 0, kV = 2.6, kA = 0;
    public static double BUFFER = 0.9;

    public static double RPM_TOLERANCE = 75;
    public static int AT_SPEED_LOOPS = 3;
    public static double STOPPED_RPM = 50;

    public static boolean ENABLE_BOOST = true;
    public static double BOOST_ON_ERROR = 300;
    public static double BOOST_OFF_ERROR = 150;
    public static double BOOST_POWER = 1.0;

    private final MotorEx motor;
    private final MotorWrapper flywheelMotor;
    private final Logger logger;

    private State state = State.STOPPED;
    private double targetRPM = 0;
    private double currentRPM = 0;
    private int inToleranceCount = 0;
    private boolean boostActive = false;

    public FlywheelFSM(HWMap hwMap, Logger logger) {
        this.logger = logger;
        motor = hwMap.getFlywheelMotor();
        flywheelMotor = new MotorWrapper(motor, true, 1, false);
        motor.setBuffer(BUFFER);
        motor.stopMotor();
    }

    public void setTargetRPM(double rpm) {
        if (!Double.isFinite(rpm)) {
            targetRPM = 0;
            return;
        }

        targetRPM = Math.max(0, rpm);
    }

    public void updateState() {
        flywheelMotor.readVelocity();
        currentRPM = flywheelMotor.getRPM();

        motor.setBuffer(BUFFER);
        flywheelMotor.setVelocityConstants(vP, vI, vD, kS, kV, kA);

        double error = targetRPM - currentRPM;

        if (targetRPM <= 0) {
            boostActive = false;
            motor.stopMotor();
        } else {
            boolean belowTarget = currentRPM < targetRPM;

            if (!ENABLE_BOOST || !belowTarget) {
                boostActive = false;
            } else {
                if (!boostActive && error >= BOOST_ON_ERROR) {
                    boostActive = true;
                }

                if (boostActive && error <= BOOST_OFF_ERROR) {
                    boostActive = false;
                }
            }

            if (boostActive) {
                motor.set(Range.clip(BOOST_POWER, 0.0, 1.0));
            } else {
                flywheelMotor.setRPM(targetRPM);
            }
        }

        if (targetRPM <= 0) {
            inToleranceCount = 0;
            state = Math.abs(currentRPM) < STOPPED_RPM
                    ? State.STOPPED
                    : State.SPINNING_UP;
        } else if (Math.abs(error) < RPM_TOLERANCE) {
            inToleranceCount++;
            state = inToleranceCount >= AT_SPEED_LOOPS
                    ? State.AT_SPEED
                    : State.SPINNING_UP;
        } else {
            inToleranceCount = 0;
            state = State.SPINNING_UP;
        }
    }

    public boolean AT_SPEED() {
        return state == State.AT_SPEED;
    }

    public boolean STOPPED() {
        return state == State.STOPPED;
    }

    public double getTargetRPM() {
        return targetRPM;
    }

    public double getCurrentRPM() {
        return currentRPM;
    }

    public boolean isBoostActive() {
        return boostActive;
    }

    public void log() {
        logger.log("<font color='yellow'>----------FLYWHEEL----------</font>", "", Logger.LogLevels.PRODUCTION);
        logger.log("Flywheel state", state, Logger.LogLevels.PRODUCTION);
        logger.log("Flywheel target RPM", String.format("%.0f", targetRPM), Logger.LogLevels.PRODUCTION);
        logger.log("Flywheel current RPM", String.format("%.0f", currentRPM), Logger.LogLevels.PRODUCTION);
        logger.log("Flywheel error", String.format("%.0f", targetRPM - currentRPM), Logger.LogLevels.DEBUG);
        logger.log("Flywheel boost", boostActive ? "FULL POWER" : "VELOCITY CONTROL", Logger.LogLevels.DEBUG);
    }
}
