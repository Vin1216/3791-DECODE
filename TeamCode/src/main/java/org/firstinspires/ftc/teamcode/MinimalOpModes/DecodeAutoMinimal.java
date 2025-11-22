package org.firstinspires.ftc.teamcode.MinimalOpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

@Disabled
@Autonomous
public class DecodeAutoMinimal extends LinearOpMode {
    private DcMotorEx FrontLeft;
    private DcMotorEx FrontRight;
    private DcMotorEx RearLeft;
    private DcMotorEx RearRight;
    private DcMotorEx IntakeMotor;
    private VisionPortal visionPortal = null;
    private AprilTagProcessor aprilTag;

    List<AprilTagDetection> currentDetections = null;
    AprilTagDetection currentDetection;

    public long exposure = (long)200;
    public int gain = 200;


    @Override
    public void runOpMode() {
        //------------------------------------------------ACTUATORS & OTHER INITIALIZATION SETUP---------------------------------------
        FrontLeft = hardwareMap.get(DcMotorEx.class, "FrontLeft");
        FrontRight = hardwareMap.get(DcMotorEx.class, "FrontRight");
        RearLeft = hardwareMap.get(DcMotorEx.class, "RearLeft");
        RearRight = hardwareMap.get(DcMotorEx.class, "RearRight");

        FrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        RearLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        DcMotorEx GreenWheel1 = hardwareMap.get(DcMotorEx.class,"flywheel1");
        DcMotorEx GreenWheel2 = hardwareMap.get(DcMotorEx.class,"flywheel2");
        GreenWheel1.setDirection(DcMotorEx.Direction.REVERSE);

        GreenWheel1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        GreenWheel2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        Servo Kicker = hardwareMap.get(Servo.class, "wshoot");
        IntakeMotor = hardwareMap.get(DcMotorEx.class, "IntakeMotor");


        ElapsedTime myTimer = new ElapsedTime();

        //------------------------------------------------CAMERA SETUP-------------------------------------------------------------
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
        //----------------------------------------------------------------RUN CODE------------------------------------------------------------------
        if(opModeIsActive()) {
            myTimer.reset();
            while(myTimer.milliseconds() < 500) {
                FrontLeft.setPower(0.2);
                FrontRight.setPower(0.2);
                RearLeft.setPower(0.2);
                RearRight.setPower(0.2);
            }
            FrontLeft.setPower(0);
            FrontRight.setPower(0);
            RearLeft.setPower(0);
            RearRight.setPower(0);
            int numTags = 0;
            while(numTags == 0 && myTimer.seconds() < 3 && !isStopRequested()) {
                currentDetections = aprilTag.getDetections();
                numTags = currentDetections.size();
            }

            if (numTags > 0 && currentDetections != null) {
                currentDetection = findCornerAprilTags(currentDetections);
            }
            if(currentDetection != null) {
                if (currentDetection.ftcPose.bearing > 0) {
                    while (!isStopRequested() && currentDetection.ftcPose.bearing > 0) {
                        FrontLeft.setPower(-0.15);
                        FrontRight.setPower(0.15);
                        RearLeft.setPower(-0.15);
                        RearRight.setPower(0.15);
                        currentDetections = aprilTag.getFreshDetections();
                        if (currentDetections != null) {
                            currentDetection = findCornerAprilTags(currentDetections);
                        }
                    }
                } else {
                    while (!isStopRequested() && currentDetection.ftcPose.bearing < 0) {
                        FrontLeft.setPower(0.15);
                        FrontRight.setPower(-0.15);
                        RearLeft.setPower(0.15);
                        RearRight.setPower(-0.15);
                        currentDetections = aprilTag.getFreshDetections();
                        if (currentDetections != null) {
                            currentDetection = findCornerAprilTags(currentDetections);
                        }
                    }
                }
            }
            else {
                myTimer.reset();
                while(myTimer.milliseconds() < 1000 && !isStopRequested()) {
                    FrontLeft.setPower(0.3);
                    FrontRight.setPower(0.3);
                    RearLeft.setPower(0.3);
                    RearRight.setPower(0.3);
                }
                FrontLeft.setPower(0);
                FrontRight.setPower(0);
                RearLeft.setPower(0);
                RearRight.setPower(0);
                requestOpModeStop();
            }
            FrontLeft.setPower(0);
            FrontRight.setPower(0);
            RearLeft.setPower(0);
            RearRight.setPower(0);
            GreenWheel1.setVelocity(2800*0.6);
            GreenWheel2.setVelocity(2800*0.6);
            for(int i = 0; i < 2 && !isStopRequested(); i++) {
                Kicker.setPosition(0);
                myTimer.reset();
                while(myTimer.milliseconds() < 1000 && !isStopRequested()) {

                }
                Kicker.setPosition(0.7);
                myTimer.reset();
                while(myTimer.milliseconds() < 1000 && !isStopRequested()) {

                }
                Kicker.setPosition(0);
            }
            myTimer.reset();
            IntakeMotor.setPower(1);
            while(myTimer.milliseconds() < 2000 && !isStopRequested()) {

            }
            IntakeMotor.setPower(0);
            Kicker.setPosition(0.7);
            myTimer.reset();
            while(myTimer.milliseconds() < 1000 && !isStopRequested()) {

            }
            Kicker.setPosition(0);
            GreenWheel1.setVelocity(0);
            GreenWheel2.setVelocity(0);
            myTimer.reset();
            while(myTimer.milliseconds() < 1000 && !isStopRequested()) {
                FrontLeft.setPower(0.3);
                FrontRight.setPower(0.3);
                RearLeft.setPower(0.3);
                RearRight.setPower(0.3);
            }
            FrontLeft.setPower(0);
            FrontRight.setPower(0);
            RearLeft.setPower(0);
            RearRight.setPower(0);
        }
    }

    private void initAprilTag() {
        // Create the AprilTag processor by using a builder.
        //TODO: Adjust camera pose to match reality
        aprilTag = new AprilTagProcessor.Builder()
                .setCameraPose(new Position(DistanceUnit.INCH,0,0,14.0,0),new YawPitchRollAngles(AngleUnit.DEGREES,0,-90,0,0))
                .build();

        // Create the WEBCAM vision portal by using a builder.
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .build();
    }
    private AprilTagDetection findCornerAprilTags(List<AprilTagDetection> detections) {
        // We prefer the red and blue tags instead of the obelisk so we TRY to find one
        for(AprilTagDetection detection: detections) {
            if((detection.id == 20 || detection.id == 24) && !isStopRequested()) {
                return detection;
            }
        }
        // Otherwise we just use whatever tag we find
        return null;
    }
}