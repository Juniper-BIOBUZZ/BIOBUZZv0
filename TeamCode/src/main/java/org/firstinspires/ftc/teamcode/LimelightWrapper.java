package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.List;

public class LimelightWrapper {

    private Limelight3A limelight;

    private boolean hasValidTarget = false;
    private double x_m = 0;
    private double y_m = 0;
    private double z_m = 0;
    private double fieldX = 0;
    private double fieldY = 0;
    private int targetID = 0;
    private double flatDistance_m = 0;
    private Telemetry telemetry;

    private static final int RED_TARGET_ID = 00;
    //NEED TO FIND #

    //Get target coordinates from SID
    private static final double GOAL_X_METERS = 0.0;
    private static final double GOAL_Y_METERS = 0.0;

    public LimelightWrapper(Limelight3A limelight3A, Telemetry telemetry) {
        limelight = limelight3A;
        limelight.setPollRateHz(100);
        limelight.start();
        limelight.pipelineSwitch(1);
        this.telemetry = telemetry;
    }

    public void update() {
        LLResult result = limelight.getLatestResult();
        telemetry.addData("is there a limelight result", !(result == null));
        telemetry.addData("is there a valid limelight result", result.isValid());

        if (result == null) {
            hasValidTarget = false;
            return;
        }

        if (!result.isValid()) {
            hasValidTarget = false;
            return;
        }

        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        //gets all reading data from camera,

        if (fiducials == null || fiducials.isEmpty()) {
            hasValidTarget = false;
            return;
        }

        LLResultTypes.FiducialResult fiducial = null;
        for (LLResultTypes.FiducialResult f : fiducials) {
            if (f.getFiducialId() == RED_TARGET_ID) {
                fiducial = f;
                break;
            }
        }
        if (fiducial == null) {
            hasValidTarget = false;
            return;
        }

        hasValidTarget = true;
        targetID = fiducial.getFiducialId();
        // Camera-Space coordinates (where you are from the April Tag like that's the center)
        x_m = fiducial.getCameraPoseTargetSpace().getPosition().x;
        y_m = fiducial.getCameraPoseTargetSpace().getPosition().y;
        z_m = fiducial.getCameraPoseTargetSpace().getPosition().z;
        flatDistance_m = 0;
        //flat distance calc^ --> pythagorean theorem
        // Field x and y
        fieldX = fiducial.getRobotPoseFieldSpace().getPosition().x;
        fieldY = fiducial.getRobotPoseFieldSpace().getPosition().y;
    }
    //getters:
    public boolean hasTarget() { return hasValidTarget; }
    public double getX() { return x_m; }
    public double getY() { return y_m; }
    public double getZ() { return z_m; }
    public double getFlatDistance() { return flatDistance_m; }
    public double getXField() { return fieldX; }
    public double getYField() { return fieldY; }
    public int getTargetID() { return targetID; }
//tell omkar your confused how the x,y,z works on limelight, show what u though and say online its different so can he explain
}