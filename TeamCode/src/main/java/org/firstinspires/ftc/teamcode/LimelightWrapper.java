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
    private int currentID = 0;
    private double flatDistance_m;
    private Telemetry telemetry;

    private final int  BLUE_TARGET_ID = 44;
    private final int  TEMP_ID = 24;

    /*
    private final double GOAL_X_METERS = 12.6; //in inches-unconverted
    private final double GOAL_Y_METERS = 12.75; //in inches-unconverted
*/ //not sure about these
    public LimelightWrapper(Limelight3A limelight3A, Telemetry telemetry) {
        limelight = limelight3A;
        limelight.setPollRateHz(100);
        limelight.start();
        limelight.pipelineSwitch(0);
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
            if (f.getFiducialId() == BLUE_TARGET_ID) {
                fiducial = f;
                break;
            }
        }
        if (fiducial == null) {
            hasValidTarget = false;
            return;
        }

        hasValidTarget = true;
        currentID = fiducial.getFiducialId();
        /*
        x_m = fiducial.getCameraPoseTargetSpace().getPosition().x;
        y_m = fiducial.getCameraPoseTargetSpace().getPosition().y;
        z_m = fiducial.getCameraPoseTargetSpace().getPosition().z;
*/

        /*
        x_m = fiducial.getRobotPoseTargetSpace().getPosition().x;
        y_m = fiducial.getRobotPoseTargetSpace().getPosition().y;
        z_m =fiducial.getRobotPoseTargetSpace().getPosition().z; // forward distance

        //flatDistance_m = Math.sqrt((x_m * x_m) + (z_m * z_m));


        double tagTiltRadians = Math.toRadians(30); //30 deg tilt fom horizontal

        //getting z down to ground
        double horizontalZ = (z_m * Math.cos(tagTiltRadians)) - (y_m * Math.sin(tagTiltRadians));
        //Now  flat distance
        flatDistance_m = Math.hypot(x_m, horizontalZ);

         */
        x_m = fiducial.getTargetPoseCameraSpace().getPosition().x;
        y_m = fiducial.getTargetPoseCameraSpace().getPosition().y;
        z_m = fiducial.getTargetPoseCameraSpace().getPosition().z; // forward distance

        int cameraPitchDegrees = 45;
        double pitchRadians = Math.toRadians(cameraPitchDegrees);

        double horizontalZ = (z_m * Math.cos(pitchRadians)) - (y_m * Math.sin(pitchRadians));
        flatDistance_m = Math.hypot(x_m, horizontalZ);

        //then make algorithm to know which one is up and down. bassically it would get the ym value
        //then compare to the known y value w/tolereance. So then u print out like two booleans
        //remember to have something so like if one is true other is false so it doesnt print true
        //true because of no update


        /*
        // Field x and y
        fieldX = fiducial.getRobotPoseFieldSpace().getPosition().x;
        fieldY = fiducial.getRobotPoseFieldSpace().getPosition().y;
        flatDistance_m = Math.sqrt((fieldX*fieldX)+(fieldY*fieldY));

         */
    }

    //getters:
    public boolean hasTarget() { return hasValidTarget; }

    public double getFlatDistance() { return flatDistance_m; }
    public double getX() { return x_m; }
    public double getY() { return y_m; }
    public double getZ() { return z_m; }
    //  public double getXField() { return fieldX; }
    //  public double getYField() { return fieldY; }
    public int getcurrentID() { return currentID; }
    public boolean correctID(){return currentID==BLUE_TARGET_ID;}
}
