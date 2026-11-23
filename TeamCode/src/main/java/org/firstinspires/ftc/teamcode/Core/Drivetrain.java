package org.firstinspires.ftc.teamcode.Core;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.function.DoubleSupplier;

public class Drivetrain {

    private final DcMotorWrapper frontRightMotor;
    private final DcMotorWrapper frontLeftMotor;
    private final DcMotorWrapper backLeftMotor;
    private final DcMotorWrapper backRightMotor;

    private final Gamepad gamepad;
    private final Telemetry telemetry;
    private final DoubleSupplier heading;

    private boolean fieldCentric;
    private double driverHeading;

    public Drivetrain(Telemetry telemetry, Gamepad gamepad,
                      MechanumHardwareMap hwMap) {
        this(telemetry, gamepad, hwMap,
                (DoubleSupplier) null, RobotSettings.Alliance.RED);
    }

    public Drivetrain(Telemetry telemetry, Gamepad gamepad,
                      MechanumHardwareMap hwMap,
                      Pinpoint pinpoint,
                      RobotSettings.Alliance alliance) {
        this(telemetry, gamepad, hwMap,
                pinpoint::getHeading, alliance);
    }

    public Drivetrain(Telemetry telemetry, Gamepad gamepad,
                      MechanumHardwareMap hwMap,
                      DoubleSupplier heading,
                      RobotSettings.Alliance alliance) {

        this.telemetry = telemetry;
        this.gamepad = gamepad;
        this.heading = heading;

        frontLeftMotor = new DcMotorWrapper(
                hwMap.getFrontLeftMotor(), 1);
        frontRightMotor = new DcMotorWrapper(
                hwMap.getFrontRightMotor(), 1);
        backLeftMotor = new DcMotorWrapper(
                hwMap.getBackLeftMotor(), 1);
        backRightMotor = new DcMotorWrapper(
                hwMap.getBackRightMotor(), 1);

        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        driverHeading = alliance == RobotSettings.Alliance.RED
                ? -90 : 90;

        fieldCentric = heading != null;
    }

    public void update() {

        double forward = -gamepad.left_stick_y;
        double strafe = gamepad.left_stick_x;
        double turn = gamepad.right_stick_x;

        if (fieldCentric && heading != null) {

            double robotHeading = heading.getAsDouble();
            double angle = Math.toRadians(driverHeading - robotHeading);

            double robotForward =
                    forward * Math.cos(angle)
                            + strafe * Math.sin(angle);

            double robotStrafe =
                    -forward * Math.sin(angle)
                            + strafe * Math.cos(angle);

            forward = robotForward;
            strafe = robotStrafe;

            telemetry.addData("Field heading", robotHeading);
            telemetry.addData("Driver heading", driverHeading);
        }

        strafe *= -1.1;

        double denominator = Math.max(
                Math.abs(forward) + Math.abs(strafe) + Math.abs(turn), 1);

        frontLeftMotor.setPower(
                (forward + strafe + turn) / denominator);

        frontRightMotor.setPower(
                (forward - strafe - turn) / denominator);

        backLeftMotor.setPower(
                (forward - strafe + turn) / denominator);

        backRightMotor.setPower(
                (forward + strafe - turn) / denominator);

        telemetry.addData("Drive mode",
                fieldCentric ? "Field Centric" : "Robot Centric");

        telemetry.addData("Robot forward", forward);
        telemetry.addData("Robot strafe", strafe);

        telemetry.addData("FLM", frontLeftMotor.getPower());
        telemetry.addData("FRM", frontRightMotor.getPower());
        telemetry.addData("BLM", backLeftMotor.getPower());
        telemetry.addData("BRM", backRightMotor.getPower());
    }

    public void resetForward() {
        if (heading != null) {
            driverHeading = heading.getAsDouble();
        }
    }

    public void setFieldCentric(boolean enabled) {
        fieldCentric = enabled && heading != null;
    }

    public void toggleFieldCentric() {
        setFieldCentric(!fieldCentric);
    }

    public boolean isFieldCentric() {
        return fieldCentric;
    }
}