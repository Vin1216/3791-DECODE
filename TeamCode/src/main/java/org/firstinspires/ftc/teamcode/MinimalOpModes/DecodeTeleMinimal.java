package org.firstinspires.ftc.teamcode.MinimalOpModes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
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
import java.util.Locale;
import java.util.concurrent.TimeUnit;

@Disabled
@TeleOp(name = "DecodeTeleOpMinimal")
public class DecodeTeleMinimal extends LinearOpMode {

    private DcMotorEx FrontLeft;
    private DcMotorEx FrontRight;
    private DcMotorEx RearLeft;
    private DcMotorEx RearRight;
    private DcMotorEx flywheel1;
    private DcMotorEx flywheel2;
    private DcMotorEx IntakeMotor;
    private Servo Kicker;

    private VisionPortal visionPortal = null;
    private AprilTagProcessor aprilTag;

    List<AprilTagDetection> currentDetections = null;
    AprilTagDetection currentDetection;


    public long exposure = (long)200;
    public int gain = 200;

    private ElapsedTime myTimer = new ElapsedTime();



    /**
     * Describe this function...
     */
    private void RunUsingEncoder() {
        FrontLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        FrontRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        RearLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        RearRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }

    /**
     * Describe this function...
     */
    private void StopAndResetEncoder() {
        FrontLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        FrontRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        RearLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        RearRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
    }

    /**
     * Describe this function...
     */
    private void DisableEncoders() {
        FrontLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        FrontRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        RearLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        RearRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }

    /**
     * This sample contains the bare minimum Blocks for any regular OpMode. The 3 blue
     * Comment Blocks show where to place Initialization code (runs once, after touching the
     * DS INIT button, and before touching the DS Start arrow), Run code (runs once, after
     * touching Start), and Loop code (runs repeatedly while the OpMode is active, namely not
     * Stopped).
     */
    @Override
    public void runOpMode() {
        double Reverse = 1;
        double SlowMode = 1;
        boolean AutoAim = false;
        double velocity = 1800;
        double maxVelocity = 2800;

        int numTags = 0;

        int colorID = 20;
//        boolean detectAprilTagsInit = true;

        double localizerDistanceToAprilTag;


        //------------------------------------------------CAMERA SETUP-------------------------------------------------------------
        initAprilTag();
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(10);
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


        //---------------------------------------------------HARDWARE INITIALIZATION---------------------------------------------------------------------

        FrontLeft = hardwareMap.get(DcMotorEx.class, "FrontLeft");
        FrontRight = hardwareMap.get(DcMotorEx.class, "FrontRight");
        RearLeft = hardwareMap.get(DcMotorEx.class, "RearLeft");
        RearRight = hardwareMap.get(DcMotorEx.class, "RearRight");
        flywheel1 = hardwareMap.get(DcMotorEx.class, "flywheel1");
        flywheel2 = hardwareMap.get(DcMotorEx.class, "flywheel2");
        IntakeMotor = hardwareMap.get(DcMotorEx.class, "IntakeMotor");
        Kicker = hardwareMap.get(Servo.class, "wshoot");

        flywheel1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        flywheel2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        flywheel1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Reversing();

        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                FrontLeft.setPower(SlowMode * (Reverse * (gamepad1.left_stick_y - (gamepad1.left_stick_x + (gamepad1.right_trigger - gamepad1.left_trigger))) - (gamepad1.right_stick_x * 0.4) ));
                FrontRight.setPower(SlowMode * (Reverse * (gamepad1.left_stick_y + (gamepad1.left_stick_x + (gamepad1.right_trigger - gamepad1.left_trigger))) + (gamepad1.right_stick_x * 0.4)));
                RearLeft.setPower(SlowMode * (Reverse * (gamepad1.left_stick_y + (gamepad1.left_stick_x + (gamepad1.right_trigger - gamepad1.left_trigger))) - (gamepad1.right_stick_x * 0.4)));
                RearRight.setPower(SlowMode * (Reverse * (gamepad1.left_stick_y - (gamepad1.left_stick_x + (gamepad1.right_trigger - gamepad1.left_trigger))) + (gamepad1.right_stick_x * 0.4)));
                if (gamepad1.right_bumper || gamepad1.b || gamepad1.left_bumper || gamepad1.a || gamepad2.x || gamepad2.y) {
                    if(gamepad1.right_bumper || gamepad1.a || gamepad2.y) {
                        IntakeMotor.setPower(1);
                    }
                    else {
                        IntakeMotor.setPower(-1);
                    }
                }
                else {
                    IntakeMotor.setPower(0);
                }
                if(gamepad1.dpadUpWasPressed()) {
                    Kicker.setPosition(0.7);
                }
                if(gamepad1.dpadDownWasPressed()) {
                    Kicker.setPosition(0);
                }
                if (gamepad1.xWasPressed()) {
                    Reverse = -1;
                }
                if (gamepad1.yWasPressed()) {
                    Reverse = 1;
                }
                if (gamepad1.dpadRightWasPressed()) {
                    SlowMode = 0.4;
                }
                if (gamepad1.dpadLeftWasPressed()) {
                    SlowMode = 1;
                }
                if (gamepad2.leftBumperWasPressed()) {
                    flywheel1.setVelocity(velocity);
                    flywheel2.setVelocity(velocity);
                }
                if (gamepad2.left_trigger > 0) {
                    flywheel1.setVelocity(0);
                    flywheel2.setVelocity(0);
                }
                if(gamepad2.dpadUpWasPressed()) {
                    velocity = maxVelocity * 0.55;
                }
                if(gamepad2.dpadRightWasPressed()) {
                    velocity = maxVelocity * 0.44;
                }
                if(gamepad2.dpadDownWasPressed()) {
                    velocity = maxVelocity * 0.3;
                }
                if(gamepad2.right_trigger > 0) {
                    visionPortal.stopStreaming();
                }
                if(gamepad2.rightBumperWasPressed()) {
                    visionPortal.resumeStreaming();
                }
                if(gamepad2.aWasPressed()) {
                    velocity -= 50;
                }
                if(gamepad2.bWasPressed()) {
                    velocity += 50;
                }
//                velocity = currentDetection.ftcPose.range;
                if(currentDetection != null) {
                    telemetry.addData("Last Detected AprilTag: ", currentDetection.id);
                }
                telemetry.addLine("-------------------POSE DATA---------------------");
                if(currentDetection != null) {
                    telemetry.addData("Bearing: ", currentDetection.ftcPose.bearing);
                    telemetry.addData("Range: ", currentDetection.ftcPose.range);
                }
                telemetry.addLine("-----------------SHOOTER STUFF--------------------");
                telemetry.addData("Target Velocity: ", velocity);
                telemetry.addLine(String.format(Locale.US, "Target Velocity %%: %.2f%%",(velocity / maxVelocity * 100) ));
                telemetry.addData("Flywheel 1: ", flywheel1.getVelocity());
                telemetry.addData("Flywheel 2: ", flywheel2.getVelocity());
                telemetry.addLine(String.format(Locale.US,"Flywheel 1 %%: %.2f%%", (flywheel1.getVelocity() / maxVelocity * 100)));
                telemetry.addLine(String.format(Locale.US,"Flywheel 2 %%: %.2f%%", (flywheel2.getVelocity() / maxVelocity * 100)));
                telemetry.update();
            }
        }
    }

    /**
     * Describe this function...
     */
    private void Reversing() {
        FrontRight.setDirection(DcMotorEx.Direction.REVERSE);
        RearRight.setDirection(DcMotorEx.Direction.REVERSE);
//        FrontLeft.setDirection(DcMotorEx.Direction.FORWARD);
//        RearLeft.setDirection(DcMotorEx.Direction.REVERSE);
        flywheel1.setDirection(DcMotorEx.Direction.REVERSE);
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

    private double getAndConvertAprilTagYaw(AprilTagDetection tag, double yaw) {
        // OBELISK CONVERSION
        if(tag.id >= 21 && tag.id <= 23) {
            if (yaw > 0) {
                return Math.toRadians(yaw - 180);
            } else {
                return Math.toRadians(yaw + 180);
            }
        }
        // RED & BLUE CONVERSION
        else if(tag.id == 20 || tag.id == 24) {
            return Math.toRadians(yaw + 90);
        }
        else {
            telemetry.addLine("ERROR GETTING YAW FROM APRILTAG");
            telemetry.update();
            return 0;
        }
    }

    private AprilTagDetection findCornerAprilTags(List<AprilTagDetection> detections) {
        // We prefer the red and blue tags instead of the obelisk so we TRY to find one
        for(AprilTagDetection detection: detections) {
            if((detection.id == 20 || detection.id == 24) && !isStopRequested()) {
                return detection;
            }
        }
        // Otherwise we just use whatever tag we find
        return detections.get(0);
    }

    private double mean(List<Double> values) {
        double total = 0;
        for (double value: values) {
            total = total + value;
        }
        return total / values.size();
    }
}