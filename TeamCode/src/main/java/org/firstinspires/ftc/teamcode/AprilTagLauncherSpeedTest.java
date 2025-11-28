package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;
@Disabled
@Config
@Autonomous
public class AprilTagLauncherSpeedTest extends LinearOpMode {

    public long exposure = (long)200;
    public int gain = 200;
    private VisionPortal visionPortal = null;
    private AprilTagProcessor aprilTag;
    public void runOpMode() {
        initAprilTag();
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }
        ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
        exposureControl.setMode(ExposureControl.Mode.Manual);
        exposureControl.setMode(ExposureControl.Mode.Manual);
        exposureControl.setExposure(exposure, TimeUnit.MICROSECONDS);

        GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
        gainControl.setGain(gain);
        waitForStart();
        if(opModeIsActive()) {
            while(opModeIsActive()) {
                List<AprilTagDetection> currentDetections = aprilTag.getDetections();
                int numTags = currentDetections.size();
                if (numTags > 0) {
                    telemetry.addData("Tag", "####### %d Detected  ######", currentDetections.size());
                    AprilTagDetection currentDetection = currentDetections.get(0);
                    telemetry.addData("RobotPose", currentDetection.robotPose);
                    telemetry.addData("FTCPose", currentDetection.ftcPose);

                } else {
                    telemetry.addData("Tag", "----------- none - ----------");
                }
                telemetry.update();
            }
        }
    }

    private void initAprilTag() {
        // Create the AprilTag processor by using a builder.
        aprilTag = new AprilTagProcessor.Builder().build();

        // Create the WEBCAM vision portal by using a builder.
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .build();
    }
}

