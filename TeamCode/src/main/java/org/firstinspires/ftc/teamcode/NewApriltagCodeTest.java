package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Autonomous
public class NewApriltagCodeTest extends LinearOpMode {
    private BNO055IMU imu;
    float Z_Rotation;
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
        Init_IMU();

        reqID = 2;
        waitForStart();
        if(opModeIsActive()) {
            MoveTowardAprilTag();
            while(opModeIsActive()) {
                telemetry.addData("ID: ", myAprilTagIdCode);
                telemetry.addLine(String.format("X: %6.1f", myAprilTagPoseX));
                telemetry.addLine(String.format("Yaw: %6.1f", myAprilTagPoseYaw));
                telemetry.addLine(String.format("Range & Bearing: %6.1f %6.1f", myAprilTagPoseRange, myAprilTagPoseBearing));
                telemetry.update();
            }
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
        double rotationStatic;
        DetectAprilTags();
        if (myAprilTagPoseX < -0.2) {
            rotationStatic = Z_Rotation + 75;
            while (Z_Rotation < rotationStatic) {
                FrontRight.setPower(0.3);
                FrontLeft.setPower(-0.3);
                RearRight.setPower(0.3);
                RearLeft.setPower(-0.3);
                IMU_Telemetry();
            }
            MoveForwardEncoder((int)Math.abs(myAprilTagPoseX * 1.4));
            rotationStatic = Z_Rotation - 75;
            while (Z_Rotation > rotationStatic) {
                FrontRight.setPower(-0.3);
                FrontLeft.setPower(0.3);
                RearRight.setPower(-0.3);
                RearLeft.setPower(0.3);
                IMU_Telemetry();
            }
        }
        else if (myAprilTagPoseX > 0.2) {
            rotationStatic = Z_Rotation - 75;
            while (Z_Rotation > rotationStatic) {
                FrontRight.setPower(-0.3);
                FrontLeft.setPower(0.3);
                RearRight.setPower(-0.3);
                RearLeft.setPower(0.3);
                IMU_Telemetry();
            }
            MoveForwardEncoder((int)Math.abs(myAprilTagPoseX * 1.4));
            rotationStatic = Z_Rotation + 75;
            while (Z_Rotation < rotationStatic) {
                FrontRight.setPower(0.3);
                FrontLeft.setPower(-0.3);
                RearRight.setPower(0.3);
                RearLeft.setPower(-0.3);
                IMU_Telemetry();
            }
        }
        FrontRight.setPower(0);
        FrontLeft.setPower(0);
        RearRight.setPower(0);
        RearLeft.setPower(0);

        if (myAprilTagPoseYaw < 0) {
            while (myAprilTagPoseYaw < 0) {
                FrontRight.setPower(0.15);
                FrontLeft.setPower(0.2);
                RearRight.setPower(0.15);
                RearLeft.setPower(0.2);
                DetectAprilTags();
            }
        }
        else {
            FrontRight.setPower(0.2);
            FrontLeft.setPower(0.15);
            RearRight.setPower(0.2);
            RearLeft.setPower(0.15);
            DetectAprilTags();
        }
        FrontRight.setPower(0);
        FrontLeft.setPower(0);
        RearRight.setPower(0);
        RearLeft.setPower(0);
        DetectAprilTags();
        MoveForwardEncoder((int)myAprilTagPoseRange - 3);
    }

    private void MoveForwardEncoder(int Distance) {
        ResetEncoder();
        tickstoDestination = (int)(Distance * ticksPerInch);
        FrontLeft.setTargetPosition(tickstoDestination);
        FrontRight.setTargetPosition(tickstoDestination);
        RearLeft.setTargetPosition(tickstoDestination);
        RearRight.setTargetPosition(tickstoDestination);
        FrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RearLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RearRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
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
    }

    private void DisableEncoders() {
        FrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RearLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RearRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    private void Init_IMU() {
        BNO055IMU.Parameters imuParameters;

        // Create new IMU Parameters object.
        imuParameters = new BNO055IMU.Parameters();
        // Use degrees as angle unit.
        imuParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        // Express acceleration as m/s^2.
        imuParameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        // Disable logging.
        imuParameters.loggingEnabled = false;
        // Initialize IMU.
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(imuParameters);
        // Prompt user to press start button.
        telemetry.addData("IMU ON", "Press start to continue...");
    }
    private void IMU_Telemetry() {
        Orientation angles;
        Acceleration gravity;

        // Get absolute orientation
        // Get acceleration due to force of gravity.
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        gravity = imu.getGravity();
        Z_Rotation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        // Display orientation info.
        telemetry.addData("rot about Z", angles.firstAngle);
        telemetry.addData("rot about Y", angles.secondAngle);
        telemetry.addData("rot about X", angles.thirdAngle);
        telemetry.update();
    }
}
