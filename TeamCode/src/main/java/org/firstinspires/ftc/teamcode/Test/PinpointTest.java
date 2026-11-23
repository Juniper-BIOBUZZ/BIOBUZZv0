package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Core.Drivetrain;
import org.firstinspires.ftc.teamcode.Core.MechanumHardwareMap;
import org.firstinspires.ftc.teamcode.Core.Pinpoint;

/*
 * Pinpoint offset test:
 *   keep the robot still, init, wait a second, then start
 *   press Cross to zero x / y / heading
 *   use the right stick to spin the robot in place half a turn (heading about 180), then stop
 *   if X moved, change the Y offset (dpad up/down), if Y moved, change the X offset (dpad left/right)
 *   it moves about twice the offset error, so change the offset by about half of how far it moved
 *   if a change makes it worse, go the other way
 *   press Cross again and repeat until X and Y stay within about half an inch after the half turn
 *   hold L1 to change by 1 mm instead of 5 mm
 *   then send the X and Y offsets
 */

@TeleOp(group = "Drivetrain", name = "Pinpoint")
public class PinpointTest extends LinearOpMode {

    private double xOffsetMm = Pinpoint.Xoffset * 25.4;
    private double yOffsetMm = Pinpoint.YOffset * 25.4;

    @Override
    public void runOpMode() {
        MechanumHardwareMap mechanumHardwareMap = new MechanumHardwareMap(hardwareMap);
        Pinpoint pinpoint = new Pinpoint(mechanumHardwareMap);
        GoBildaPinpointDriver odo = mechanumHardwareMap.getOdo();
        Drivetrain drivetrain = new Drivetrain(telemetry, gamepad1, mechanumHardwareMap);
        odo.setOffsets(xOffsetMm, yOffsetMm, DistanceUnit.MM);

        boolean lastCross = false, lastUp = false, lastDown = false, lastLeft = false, lastRight = false;

        while (opModeInInit()) {
            pinpoint.update();
            telemetry.addData("Status", pinpoint.getStatus());
            telemetry.update();
        }

        while (opModeIsActive()) {
            pinpoint.update();

            double step = gamepad1.left_bumper ? 1 : 5;
            boolean changed = false;
            if (gamepad1.dpad_right && !lastRight) { xOffsetMm += step; changed = true; }
            if (gamepad1.dpad_left && !lastLeft) { xOffsetMm -= step; changed = true; }
            if (gamepad1.dpad_up && !lastUp) { yOffsetMm += step; changed = true; }
            if (gamepad1.dpad_down && !lastDown) { yOffsetMm -= step; changed = true; }
            if (changed) odo.setOffsets(xOffsetMm, yOffsetMm, DistanceUnit.MM);
            if (gamepad1.a && !lastCross) {
                odo.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0));
            }
            lastCross = gamepad1.a;
            lastUp = gamepad1.dpad_up;
            lastDown = gamepad1.dpad_down;
            lastLeft = gamepad1.dpad_left;
            lastRight = gamepad1.dpad_right;

            drivetrain.update();

            telemetry.addData("Status", pinpoint.getStatus());
            telemetry.addData("X offset mm (dpad L/R)", String.format("%.1f", xOffsetMm));
            telemetry.addData("Y offset mm (dpad up/down)", String.format("%.1f", yOffsetMm));
            telemetry.addData("X pos", String.format("%.2f", pinpoint.getX()));
            telemetry.addData("Y pos", String.format("%.2f", pinpoint.getY()));
            telemetry.addData("Heading", String.format("%.1f", pinpoint.getHeading()));
            telemetry.addData("Raw x encoder", odo.getEncoderX());
            telemetry.addData("Raw y encoder", odo.getEncoderY());
            telemetry.update();
        }
    }
}
