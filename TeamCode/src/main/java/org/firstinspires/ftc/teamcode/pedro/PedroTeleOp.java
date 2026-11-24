package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.ManualDrive;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Pedro TeleOp")
public class PedroTeleOp extends LinearOpMode {

    private Follower follower;
    private ElapsedTime loopTimer = new ElapsedTime(); // Create the timer


    @Override
    public void runOpMode() {
        follower = Constants.create(hardwareMap);
        follower.setPose(new Pose(72, 72, 0));
        double loopTimeMS = loopTimer.milliseconds();
        loopTimer.reset();
        waitForStart();
        while (opModeIsActive()) {
            follower.update();

            DrivePowers powers = ManualDrive.fieldCentric(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, follower.pose().heading());
            follower.manual(powers);
            // ManualDrive.driveOrHold(follower, powers); -making it lock up;

            Pose robotPose = follower.pose();
            //add relocalization?



            //pedro pathing graphs one quadrant only so its (0,0) to (144,144).
            telemetry.addData("Robot X", robotPose.x());
            telemetry.addData("Robot Y", robotPose.y());
            telemetry.addData("Robot Heading", Math.toDegrees(robotPose.heading()));
            telemetry.addData("Follower Mode", follower.mode());
            telemetry.addData("Loop Time in MS", loopTimeMS);
           // telemetry.addData("Loop Time:", follower.)
            telemetry.update();
        }
    }
}

