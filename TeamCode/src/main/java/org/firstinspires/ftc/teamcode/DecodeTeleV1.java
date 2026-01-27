package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
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

@TeleOp(name = "DecodeTeleOpV1")
public class DecodeTeleV1 extends LinearOpMode {
    boolean flywheelsActive = false;

    public int colorID = 20;

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


    public long exposure = (long)500;
    public int gain = 200;

    private ElapsedTime myTimer = new ElapsedTime();
    private ElapsedTime loopTime = new ElapsedTime();

    private double autoAimAdjustmentAngle;

    DcMotorEx motor;

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
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0,0,0));;
        TrajectoryActionBuilder traj;
        PoseVelocity2d currentPose;
        Pose2d startpose = null;


        double Reverse = -1;
        double SlowMode = 1;
        boolean AutoAim = false;
        double velocity = 840;
        double maxVelocity = 2800;

        boolean detectAprilTagsInit = true;

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

        //---------------------------------------------SETUP PROMPTS------------------------------------------------------------------------
        // tbh this is kinda useless but hey it looks cool
//        while(!isStopRequested()) {
//            telemetry.addLine("Wot color");
//            telemetry.addLine("Blue                  Red");
//            if(colorID == 20) {
//                telemetry.addLine("  ^");
//            }
//            else {
//                telemetry.addLine("                               ^");
//            }
//            telemetry.update();
//
//            if(gamepad1.dpadLeftWasPressed() && colorID == 24) {
//                colorID = 20;
//            }
//            else if(gamepad1.dpadRightWasPressed() && colorID == 20) {
//                colorID = 24;
//            }
//
//            if(gamepad1.aWasPressed()) {
//                break;
//            }
////            sleep(20);
//        }

//        while(!isStopRequested()) {
//            telemetry.addLine("Find AprilTag rn?????");
//            telemetry.addLine("A for yes          B for no");
//            telemetry.update();
//
//            if(gamepad1.aWasPressed()) {
//                break;
//            }
//            else if(gamepad1.bWasPressed()) {
//                detectAprilTagsInit = false;
//                break;
//            }
////            sleep(20);
//        }

        //------------------------------------------------APRILTAG DETECTION & LOCALIZATION-----------------------------------------------------
          int numTags;
//        List<Double> totalRobotX = new ArrayList<>();
//        List<Double> totalRobotY = new ArrayList<>();
//        List<Double> totalRobotYaw = new ArrayList<>();
//        currentDetections = aprilTag.getDetections();
//        if(detectAprilTagsInit) {
//            numTags = currentDetections.size();
//        }
//        else {
//            numTags = -1;
//        }
//
//        myTimer.reset();
//        while(numTags == 0 && myTimer.seconds() < 3 && !isStopRequested()) {
//            currentDetections = aprilTag.getDetections();
//            numTags = currentDetections.size();
//        }
//
//        if (numTags > 0) {
//            currentDetection = findCornerAprilTags(currentDetections);
//
//            if (!isStopRequested()) {
//                telemetry.addData("Tag", "####### %d Detected  ######", currentDetections.size());
//                telemetry.addData("ID", currentDetection.id);
//                telemetry.addData("RobotPose", currentDetection.robotPose);
//                telemetry.addData("FTCPose", currentDetection.ftcPose);
//                totalRobotX.add(currentDetection.robotPose.getPosition().x);
//                totalRobotY.add(currentDetection.robotPose.getPosition().y);
//            }
//            myTimer.reset();
//            while (myTimer.seconds() < 2 && !isStopRequested()) {
//                currentDetections = aprilTag.getFreshDetections();
//                if (currentDetections != null) {
//                    if (currentDetections.isEmpty()) {
//                        continue;
//                    }
//                    currentDetection = findCornerAprilTags(currentDetections);
//                    if(currentDetection.id != 20 && currentDetection.id != 24) {
//                        continue;
//                    }
////                    currentDetection = currentDetections.get(0);
//                    telemetry.addData("Updating X... ", currentDetection.robotPose.getPosition().x);
//                    totalRobotX.add(currentDetection.robotPose.getPosition().x);
//                    telemetry.addData("Updating Y...", currentDetection.robotPose.getPosition().y);
//                    totalRobotY.add(currentDetection.robotPose.getPosition().y);
//                    telemetry.addData("Updating Yaw...", currentDetection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES));
//                    totalRobotYaw.add(currentDetection.robotPose.getOrientation().getYaw());
//                    telemetry.update();
//                }
//            }
//            startpose = new Pose2d(mean(totalRobotX), mean(totalRobotY), getAndConvertAprilTagYaw(currentDetection,mean(totalRobotYaw)));
//
//        } else {
//            telemetry.addData("Tag", "----------- none - ----------");
//        }

        if(startpose == null) {
            //TODO: Add the estimated end of the autonomous
            startpose = new Pose2d(0, 0, Math.toRadians(180));
        }
        drive.localizer.setPose(startpose);

        telemetry.addData("FinalPoseX: ",startpose.position.x);
        telemetry.addData("FinalPoseY: ",startpose.position.y);
        telemetry.addData("FinalYaw: ", startpose.heading.log());
        telemetry.update();

        //---------------------------------------------------HARDWARE INITIALIZATION---------------------------------------------------------------------

        FrontLeft = drive.leftFront;
        FrontRight = drive.rightFront;
        RearLeft = drive.leftBack;
        RearRight = drive.rightBack;
//        flywheel1 = hardwareMap.get(DcMotorEx.class, "flywheel1");
//        flywheel2 = hardwareMap.get(DcMotorEx.class, "flywheel2");
        IntakeMotor = hardwareMap.get(DcMotorEx.class, "IntakeMotor");
        Kicker = hardwareMap.get(Servo.class, "wshoot");

//        flywheel1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
//        flywheel2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
//        flywheel1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        flywheel2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        Reversing();

        FlywheelCustomPID flywheels = new FlywheelCustomPID(hardwareMap);

        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                loopTime.reset();
                currentPose = drive.localizer.update();

                FrontLeft.setPower(SlowMode * (Reverse * (gamepad1.left_stick_y - (gamepad1.left_stick_x + (gamepad1.right_trigger - gamepad1.left_trigger)) * 0.75) + (gamepad1.right_stick_x * 0.6)));
                FrontRight.setPower(SlowMode * (Reverse * (gamepad1.left_stick_y + (gamepad1.left_stick_x + (gamepad1.right_trigger - gamepad1.left_trigger)) * 0.75) - (gamepad1.right_stick_x * 0.6)));
                RearLeft.setPower(SlowMode * (Reverse * (gamepad1.left_stick_y + (gamepad1.left_stick_x + (gamepad1.right_trigger - gamepad1.left_trigger)) * 0.75) + (gamepad1.right_stick_x * 0.6)));
                RearRight.setPower(SlowMode * (Reverse * (gamepad1.left_stick_y - (gamepad1.left_stick_x + (gamepad1.right_trigger - gamepad1.left_trigger)) * 0.75) - (gamepad1.right_stick_x * 0.6)));
                if (gamepad1.right_bumper || gamepad1.b || gamepad1.a || gamepad2.x || gamepad2.y) {
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
                if(gamepad1.dpadUpWasPressed() || gamepad2.rightBumperWasPressed()) {
                    Kicker.setPosition(0.7);
                }
                if(gamepad1.dpadDownWasPressed() || gamepad2.right_trigger > 0) {
                    Kicker.setPosition(0);
                }
                if (gamepad1.xWasPressed()) {
                    flywheels.setVelocity(0);
                    flywheelsActive = false;
                    Reverse = 1;
                }
                if (gamepad1.yWasPressed()) {
                    Reverse = -1;
                }
                if (gamepad1.dpadRightWasPressed()) {
                    SlowMode = 0.75;
                }
                if (gamepad1.dpadLeftWasPressed()) {
                    SlowMode = 1;
                }
                if (gamepad2.leftBumperWasPressed()) {
                    flywheelsActive = true;
                }
                if (gamepad2.left_trigger > 0) {
                    flywheels.setVelocity(0);
                    flywheelsActive = false;
                }
                if(flywheelsActive) {
                    flywheels.setVelocity(velocity);
                }
                else if(flywheels.flywheel1.getVelocity() != 0) {
                    flywheels.setVelocity(0);
                }
                if(gamepad2.dpadUpWasPressed()) {
                    velocity = 1120;
                }
                if(gamepad2.dpadRightWasPressed()) {
                    velocity = 1040;
                }
                if(gamepad2.dpadDownWasPressed()) {
                    velocity = 940;
                }
//                if(gamepad2.right_trigger > 0) {
//                    visionPortal.stopStreaming();
//                }
//                if(gamepad2.rightBumperWasPressed()) {
//                    visionPortal.resumeStreaming();
//                }
                if(gamepad2.aWasPressed()) {
                    velocity -= 20;
                }
                if(gamepad2.bWasPressed()) {
                    velocity += 20;
                }
                if (visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING && currentPose.linearVel.equals(new Vector2d(0,0)) && currentPose.angVel == 0) {
                    telemetry.addLine("Camera: ON");
                    currentDetections = aprilTag.getDetections();
                    numTags = currentDetections.size();
                    if (numTags > 0) {
                        currentDetection = findCornerAprilTags(currentDetections);
                        if (currentDetection.id == 20 || currentDetection.id == 24) {
                            colorID = currentDetection.id;
                            drive.localizer.setPose(new Pose2d(currentDetection.robotPose.getPosition().x,
                                    currentDetection.robotPose.getPosition().y,
                                    Math.toRadians(getAndConvertAprilTagYaw(currentDetection, currentDetection.robotPose.getOrientation().getYaw()))));
                            AutoAim = true;
                            telemetry.addLine("AutoAim ACTIVE");
                        } else {
                            AutoAim = false;
                            telemetry.addLine("AutoAim NOOOOOOOOOOOOOOO");
                        }
                    }
                    else {
                        AutoAim = false;
                        telemetry.addLine("AutoAim NOOOOOOOOOOOOOOO");
                    }
                }
                else {
                    if(visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
                        telemetry.addLine("Camera: OFF");
                    }
                    else {
                        telemetry.addLine("Camera: OFF");
                    }
                    AutoAim = false;
                    telemetry.addLine("AutoAim NOOOOOOOOOOOOOOO");
                }


//                if(gamepad2.yWasPressed()) {
//                    telemetry.addLine("UPDATING POSE FROM LAST DETECTED APRILTAG");
//                    telemetry.update();
//                    drive.localizer.setPose(new Pose2d(currentDetection.robotPose.getPosition().x,
//                            currentDetection.robotPose.getPosition().y,
//                            Math.toRadians(getAndConvertAprilTagYaw(currentDetection, currentDetection.robotPose.getOrientation().getYaw()))));
//                }
//                if(currentPose.linearVel.equals(0))
                if(gamepad1.left_bumper && AutoAim && (currentDetection != null)) {
//                    traj = drive.actionBuilder(drive.localizer.getPose())
//                                .turn(Math.toRadians(currentDetection.ftcPose.bearing) * 0.95);
                    if(colorID == 20) {
                        autoAimAdjustmentAngle = getAutoAimAdjustmentAngle(
                                currentDetection.ftcPose.range,
                                Math.sqrt(Math.pow(-72 - drive.localizer.getPose().position.x, 2) + Math.pow(-72 - drive.localizer.getPose().position.y, 2))
                        );
                    }
                    else {
                        autoAimAdjustmentAngle = getAutoAimAdjustmentAngle(
                                currentDetection.ftcPose.range,
                                Math.sqrt(Math.pow(-72 - drive.localizer.getPose().position.x, 2) + Math.pow(72 - drive.localizer.getPose().position.y, 2))
                        );
                    }
                    if(!Double.isNaN(autoAimAdjustmentAngle)) {
                        traj = drive.actionBuilder(drive.localizer.getPose()).turn(Math.toRadians(currentDetection.ftcPose.bearing - autoAimAdjustmentAngle));
                        Actions.runBlocking(traj.build());
                    }
//                    telemetry.addData("autoAimAdjustmentAngle", autoAimAdjustmentAngle);
                }

//                velocity = currentDetection.ftcPose.range;
                if(!currentDetections.isEmpty()) {
                    telemetry.addData("Last Detected AprilTag: ", currentDetection.id);
                }
                telemetry.addLine("-------------------POSE DATA---------------------");
                telemetry.addData("PoseX: ", drive.localizer.getPose().position.x);
                telemetry.addData("PoseY: ", drive.localizer.getPose().position.y);
                telemetry.addData("Rotation: ", Math.toDegrees(drive.localizer.getPose().heading.log()));
                if(!currentDetections.isEmpty()) {
                    telemetry.addData("Bearing: ", currentDetection.ftcPose.bearing);
                    telemetry.addData("Range: ", currentDetection.ftcPose.range);
                }

                if(colorID == 20) {
                    localizerDistanceToAprilTag = Math.sqrt(Math.pow(-56 - drive.localizer.getPose().position.x, 2) + Math.pow(-56 - drive.localizer.getPose().position.y, 2));
                }
                else {
                    localizerDistanceToAprilTag = Math.sqrt(Math.pow(-56 - drive.localizer.getPose().position.x, 2) + Math.pow(56 - drive.localizer.getPose().position.y, 2));
                }
                telemetry.addData("Calculated Distance to AprilTag: ", localizerDistanceToAprilTag);

                telemetry.addLine("-----------------SHOOTER STUFF--------------------");
                telemetry.addData("Target Velocity: ", velocity);
                telemetry.addLine(String.format(Locale.US, "Target Velocity %%: %.2f%%",(velocity / maxVelocity * 100) ));
                telemetry.addData("Flywheel 1: ", flywheels.flywheel1.getVelocity());
                telemetry.addData("Flywheel 2: ", flywheels.flywheel2.getVelocity());
                telemetry.addLine(String.format(Locale.US,"Flywheel 1 %%: %.2f%%", (flywheels.flywheel1.getVelocity() / maxVelocity * 100)));
                telemetry.addLine(String.format(Locale.US,"Flywheel 2 %%: %.2f%%", (flywheels.flywheel2.getVelocity() / maxVelocity * 100)));
                telemetry.addLine("--------------------------------------------------");
                telemetry.addData("Loop Time (ms): ", loopTime.milliseconds());
                telemetry.update();
            }
        }
    }

    /**
     * Describe this function...
     */
    private void Reversing() {
//        FrontRight.setDirection(DcMotorEx.Direction.REVERSE);
//        RearRight.setDirection(DcMotorEx.Direction.FORWARD);
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

    private double getAutoAimAdjustmentAngle(double aprilTagDistance, double cornerDistance) {
        double aprilTagDistanceSquared = Math.pow(aprilTagDistance, 2);
        double cornerDistanceSquared = Math.pow(cornerDistance, 2);
        double aprilTagtoCornerSquared = 512.0;
        double angle = Math.acos((aprilTagtoCornerSquared - aprilTagDistanceSquared - cornerDistanceSquared) / (-2 * aprilTagDistance * cornerDistance));
        return angle * 0.5;
//        if(angle > 0) {
//        }
//        else {
//            return 0;
//        }
    }
}