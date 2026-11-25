package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class LimelightTestClass extends LinearOpMode {

    private Limelight3A limelight3A;
    private LimelightWrapper limelightWrapper;

    @Override
    public void runOpMode(){
        limelight3A = hardwareMap.get(Limelight3A.class, "limelight");

        limelightWrapper = new LimelightWrapper(limelight3A, telemetry);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            limelightWrapper.update();

            if (limelightWrapper.hasTarget()) {
                telemetry.addData("Target Status", "Target Found (ID: %d)", limelightWrapper.getcurrentID());
                telemetry.addData("Correct Target?",limelightWrapper.correctID());

                telemetry.addData("Flat Line Distance", "%.2f meters", limelightWrapper.getFlatDistance());

                //limelight pos
                telemetry.addData("Robot X-relative to CAM.", "%.2f meters", limelightWrapper.getX());
                telemetry.addData("Robot Z-relative to CAM.", "%.2f meters", limelightWrapper.getZ());
                telemetry.addData("Robot Y-relative to CAM.", "%.2f meters", limelightWrapper.getY());
            } else {
                telemetry.addData("Target Status", "No Target Detected");
            }

            telemetry.update();
        }
    }
}