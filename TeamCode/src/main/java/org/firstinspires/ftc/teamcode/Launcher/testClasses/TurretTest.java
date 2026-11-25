package org.firstinspires.ftc.teamcode.Launcher.testClasses;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.util.Timing;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Core.HWMap;
import org.firstinspires.ftc.teamcode.Core.MotorWrapper;

//CHANGE TO FSM LATER

import java.util.concurrent.TimeUnit;
@Config
public class TurretTest {
    HWMap hwMap;
    MotorWrapper turretMotor;
    public static double targetAngle;

    private PIDController pidController;
    public static double TOLERANCE = 3;
    public static double P_Counter = 0.04;
    public static double P_clock = 0.02;
    public static double P = 0.01, I = 0.0, D = 0, F = 0.2;
    public static double gearRatio = 13.0 / 65.0;

    public static double UPPER_HARD_STOP = 0;
    public static double LOWER_HARD_STOP = -265;

    public static double POWER_CAP = 0.25;
    Telemetry telemetry;

    Timing.Timer timer;
    public static long sleepTime = 45;

    public TurretTest( HWMap hwMap, Telemetry telemetry) {
        timer = new Timing.Timer(3000000, TimeUnit.MILLISECONDS);
        turretMotor = new MotorWrapper(hwMap.getTurretMotor(), false, gearRatio, false);
        turretMotor.resetEncoder();
        this.telemetry = telemetry;
        pidController = new PIDController(P, I, D);

    }

    public void updatePID() {

        pidController.setPID(P, I, D);
        pidController.setTolerance(TOLERANCE);
        turretMotor.readPosition();
        if (targetAngle > UPPER_HARD_STOP) {
            targetAngle = UPPER_HARD_STOP;
        } else if (targetAngle < LOWER_HARD_STOP) {
            targetAngle = LOWER_HARD_STOP;
        }

        double currentPos = turretMotor.getScaledPos();
        double error = targetAngle - currentPos;

        if (error < 0) {
            P = P_Counter;
        } else {
            P = P_clock;
        }
        double power = pidController.calculate(currentPos, targetAngle);
        if (Math.abs(error) >= TOLERANCE) {
            power = power + (F * Math.signum(error));
        }
        if (Math.abs(power) > POWER_CAP) {
            double signPower = Math.signum(power);
            power = signPower * POWER_CAP;
        }

        telemetry.addData("Turret Target Angle", targetAngle);
        telemetry.addData("Turret Motor Target Angle", targetAngle * (109.0 / 16.0));
        telemetry.addData("Turret Motor Current Angle", turretMotor.getAngle());
        telemetry.addData("Turret Current Angle", turretMotor.getScaledPos());
        telemetry.addData("Ticks Per Rev", turretMotor.getTicksPerRev());
        telemetry.addData("Current Pos Ticks", turretMotor.readandGetPosTicks());
        telemetry.addData("Loop Time", timer.elapsedTime());

        turretMotor.set(power);
    }
}
