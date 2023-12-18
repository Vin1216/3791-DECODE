package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public class NewApriltagCodeTest extends LinearOpMode {
    DcMotor FrontRight;
    DcMotor FrontLeft;
    DcMotor RearRight;
    DcMotor RearLeft;
    AprilTagProcessor myAprilTagProcessor;

    List<AprilTagDetection> myAprilTagDetections;
    AprilTagDetection myAprilTagDetection;
    int myAprilTagIdCode;
    double myAprilTagPoseX;
    double myAprilTagPoseBearing;
    double myAprilTagPoseRange;
    double myAprilTagPoseYaw;
    int reqID;

    int tickstoDestination;
    double ticksPerInch;


    public void runOpMode() {
        FrontRight = hardwareMap.get(DcMotor.class, "FrontRight");
        FrontLeft = hardwareMap.get(DcMotor.class, "FrontLeft");
        RearRight = hardwareMap.get(DcMotor.class, "RearRight");
        RearLeft = hardwareMap.get(DcMotor.class, "RearLeft");
        FrontRight.setDirection(DcMotor.Direction.REVERSE);
        FrontLeft.setDirection(DcMotor.Direction.REVERSE);
        RearRight.setDirection(DcMotor.Direction.REVERSE);
        RearLeft.setDirection(DcMotor.Direction.REVERSE);
        int ticksperRevolution = 480;
        double wheelCircumference = 12.56;
        ticksPerInch = ticksperRevolution / wheelCircumference;
        InitVisionPortal();

        reqID = 2;
        waitForStart();
        if(opModeIsActive()) {
            MoveTowardAprilTag();
        }
    }

    void InitVisionPortal() {
        VisionPortal myVisionPortal;
        myAprilTagProcessor = AprilTagProcessor.easyCreateWithDefaults();
        myVisionPortal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "WebCam"), myAprilTagProcessor);
    }

    private void DetectAprilTags() {
        // Get a list containing the latest detections, which may be stale.
        myAprilTagDetections = myAprilTagProcessor.getDetections();
        for (AprilTagDetection myAprilTagDetection_item : myAprilTagDetections) {
            myAprilTagDetection = myAprilTagDetection_item;
            if (myAprilTagDetection.metadata != null) {
                myAprilTagIdCode = myAprilTagDetection.id;
                if (myAprilTagIdCode == reqID) {
                    myAprilTagPoseX = myAprilTagDetection.ftcPose.x;
                    myAprilTagPoseBearing = myAprilTagDetection.ftcPose.bearing;
                    myAprilTagPoseRange = myAprilTagDetection.ftcPose.range;
                    myAprilTagPoseYaw = myAprilTagDetection.ftcPose.yaw;
                    break;
                }
            }
        }
    }

    private void MoveTowardAprilTag() {
        DetectAprilTags();
        if (myAprilTagPoseX < 0) {
            while (myAprilTagPoseX < 0) {
                FrontRight.setPower(0.5);
                FrontLeft.setPower(-0.5);
                RearRight.setPower(-0.5);
                RearLeft.setPower(0.5);
                DetectAprilTags();
            }
        }
        else {
            while (myAprilTagPoseX > 0) {
                FrontRight.setPower(-0.5);
                FrontLeft.setPower(0.5);
                RearRight.setPower(0.5);
                RearLeft.setPower(-0.5);
                DetectAprilTags();
            }
        }
        FrontRight.setPower(0);
        FrontLeft.setPower(0);
        RearRight.setPower(0);
        RearLeft.setPower(0);

        if (myAprilTagPoseYaw < 0) {
            while (myAprilTagPoseYaw < 0) {
                FrontRight.setPower(0.15);
                FrontLeft.setPower(0.4);
                RearRight.setPower(0.15);
                RearLeft.setPower(0.4);
                DetectAprilTags();
            }
        }
        else {
            FrontRight.setPower(0.4);
            FrontLeft.setPower(0.15);
            RearRight.setPower(0.4);
            RearLeft.setPower(0.15);
            DetectAprilTags();
        }
        FrontRight.setPower(0);
        FrontLeft.setPower(0);
        RearRight.setPower(0);
        RearLeft.setPower(0);

        MoveForwardEncoder((int)myAprilTagPoseRange - 2);
    }

    private void MoveForwardEncoder(int Distance) {
        ResetEncoder();
        tickstoDestination = (int)(Distance * ticksPerInch);
        FrontLeft.setTargetPosition(tickstoDestination);
        FrontRight.setTargetPosition(tickstoDestination);
        RearLeft.setTargetPosition(tickstoDestination);
        RearRight.setTargetPosition(tickstoDestination);
        FrontLeft.setPower(0.5);
        FrontRight.setPower(0.5);
        RearLeft.setPower(0.5);
        RearRight.setPower(0.5);
        while (FrontRight.isBusy()) {
        }
        FrontLeft.setPower(0);
        FrontRight.setPower(0);
        RearLeft.setPower(0);
        RearRight.setPower(0);
        DisableEncoders();
    }

    private void ResetEncoder() {
        FrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RearLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RearRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RearLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RearRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RearLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RearRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    private void DisableEncoders() {
        FrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RearLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RearRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

}
