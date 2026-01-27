package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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
import java.util.concurrent.TimeUnit;

@Autonomous
public class DecodeAutoBLUECLOSEV1 extends LinearOpMode {
    private VisionPortal visionPortal = null;
    private AprilTagProcessor aprilTag;

    List<AprilTagDetection> currentDetections = null;

    public long exposure = (long)200;
    public int gain = 200;
    ElapsedTime myTimer = new ElapsedTime();

    DcMotorEx IntakeMotor;
    DcMotorEx GreenWheel1;
    DcMotorEx GreenWheel2;
    Servo Kicker;
    FlywheelCustomPID flywheels;

    double maxVelocity = 2800;

    @Override
    public void runOpMode() {
        PoseVelocity2d currentPose;
        int numTags = 0;
        //------------------------------------------------ACTUATORS & OTHER INITIALIZATION SETUP---------------------------------------
        IntakeMotor = hardwareMap.get(DcMotorEx.class, "IntakeMotor");
//        GreenWheel1 = hardwareMap.get(DcMotorEx.class,"flywheel1");
//        GreenWheel2 = hardwareMap.get(DcMotorEx.class,"flywheel2");
//        GreenWheel1.setDirection(DcMotorEx.Direction.REVERSE);
//
//        GreenWheel1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        GreenWheel2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        flywheels = new FlywheelCustomPID(hardwareMap);
        Kicker = hardwareMap.get(Servo.class, "wshoot");

        MecanumDrive drive =  new MecanumDrive(hardwareMap, new Pose2d(0,0,0));
        Pose2d startpose = null;

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

        //------------------------------------------------APRILTAG DETECTION & LOCALIZATION-----------------------------------------------------
//        List<Double> totalRobotX = new ArrayList<>();
//        List<Double> totalRobotY = new ArrayList<>();
//        List<Double> totalRobotYaw = new ArrayList<>();
//        currentDetections = aprilTag.getDetections();
//        numTags = currentDetections.size();
//        myTimer.reset();
//        while(numTags == 0 && myTimer.seconds() < 5 && !isStopRequested()) {
//            currentDetections = aprilTag.getDetections();
//            numTags = currentDetections.size();
//        }
//
//        if (numTags > 0) {
//            AprilTagDetection currentDetection = findCornerAprilTags(currentDetections);
//
//            if (!isStopRequested()) {
//                telemetry.addData("Tag", "####### %d Detected  ######", currentDetections.size());
//                telemetry.addData("ID", currentDetection.id);
//                telemetry.addData("RobotPose", currentDetection.robotPose);
//                telemetry.addData("FTCPose", currentDetection.ftcPose);
//                totalRobotX.add(currentDetection.robotPose.getPosition().x);
//                totalRobotY.add(currentDetection.robotPose.getPosition().y);
//
//                switch (currentDetection.id) {
////                    case 20:
////                        DecodeTeleV1.colorID = 20;
//                    case 21:
//                        telemetry.addLine("GPP");
//                        break;
//                    case 22:
//                        telemetry.addLine("PGP");
//                        break;
//                    case 23:
//                        telemetry.addLine("PPG");
//                        break;
////                    case 24:
////                        DecodeTeleV1.colorID = 24;
//                }
//            }
//            myTimer.reset();
//            while (myTimer.seconds() < 3 && !isStopRequested()) {
//                currentDetections = aprilTag.getFreshDetections();
//                if (currentDetections != null) {
//                    if (currentDetections.isEmpty()) {
//                        continue;
//                    }
//                    currentDetection = findCornerAprilTags(currentDetections);
//                    if(currentDetection.id <= 23 && currentDetection.id >= 21) {
//                        continue;
//                    }
////                    currentDetection = currentDetections.get(0);
//                    telemetry.addData("Updating X... ", currentDetection.robotPose.getPosition().x);
//                    totalRobotX.add(currentDetection.robotPose.getPosition().x);
//                    telemetry.addData("Updating Y...", currentDetection.robotPose.getPosition().y);
//                    totalRobotY.add(currentDetection.robotPose.getPosition().y);
//                    telemetry.addData("Updating Yaw...", currentDetection.robotPose.getOrientation().getYaw());
//                    totalRobotYaw.add(currentDetection.robotPose.getOrientation().getYaw());
//                    telemetry.update();
//                }
//            }
//            startpose = new Pose2d(mean(totalRobotX), mean(totalRobotY), getAndConvertAprilTagYaw(currentDetection,mean(totalRobotYaw)));
//
//        } else {
//            telemetry.addData("Tag", "----------- none - ----------");
//        }
//
//        //------------------------------------------------ROADRUNNER FIRST TRAJECTORY-------------------------------------------------------------
//        //backup in case no apriltag was detected
//        if(startpose == null) {
//            startpose = new Pose2d(-56 + 6.36, -56 + 6.36, Math.toRadians(225));
//        }
        startpose = new Pose2d(-56 + 6.36, -56 + 6.36, Math.toRadians(225));
        drive.localizer.setPose(startpose);

        TrajectoryActionBuilder PreloadTraj = drive.actionBuilder(startpose)
                .splineToConstantHeading(new Vector2d(-15,-15),Math.toRadians(45));
        telemetry.addData("FinalPoseX",startpose.position.x);
        telemetry.addData("FinalPoseY",startpose.position.y);
        telemetry.addData("FinalYaw", startpose.heading.log());
        telemetry.update();
        waitForStart();
        //----------------------------------------------------------------RUN CODE------------------------------------------------------------------
        if(opModeIsActive()) {
            Actions.runBlocking(PreloadTraj.build());
            launchThreeFar();
            TrajectoryActionBuilder firstSet = PreloadTraj.fresh()
                    .splineToLinearHeading(new Pose2d(-10,-30,Math.toRadians(90)),Math.toRadians(270));

            Actions.runBlocking(firstSet.build());
            IntakeMotor.setPower(0.75);
            Actions.runBlocking(firstSet.fresh().setReversed(true).lineToY(-56, new TranslationalVelConstraint(60)).build());
            IntakeMotor.setPower(0);
            Actions.runBlocking(
                    drive.actionBuilder(drive.localizer.getPose())
                            .splineToLinearHeading(new Pose2d(-15,-15,Math.toRadians(225)), Math.toRadians(225))
                            .build()
            );

            launchThreeFar();

            TrajectoryActionBuilder secondSet = drive.actionBuilder(drive.localizer.getPose())
                    .splineToLinearHeading(new Pose2d(14,-30,Math.toRadians(90)),Math.toRadians(270));
            Actions.runBlocking(secondSet.build());
            IntakeMotor.setPower(0.75);
            Actions.runBlocking(drive.actionBuilder(drive.localizer.getPose()).setReversed(true).lineToY(-66, new TranslationalVelConstraint(60)).build());
            IntakeMotor.setPower(0);
            Actions.runBlocking(
                    drive.actionBuilder(drive.localizer.getPose())
                            .splineToLinearHeading(new Pose2d(-15,-15,Math.toRadians(225)), Math.toRadians(225))
                            .build()
            );
            launchThreeFar();


            Actions.runBlocking(drive.actionBuilder(drive.localizer.getPose()).splineToConstantHeading(new Vector2d(6,-24),Math.toRadians(270)).build());
        }
    }

    private void launchThreeFar() {
        ElapsedTime targetTimer = new ElapsedTime();
        double[] powers = {860,880,860};
        double target;
        boolean targetreached = false;

        for(int i = 0; i < 3; i++) {
            target = powers[i];
//            GreenWheel1.setVelocity(maxVelocity * powers[i]);
//            GreenWheel2.setVelocity(maxVelocity * powers[i]);
            targetTimer.reset();
            while(flywheels.flywheel1.getVelocity() != target && !isStopRequested() && targetTimer.milliseconds() <= 500) {
                flywheels.setVelocity(target);
                if(flywheels.flywheel1.getVelocity() == target && targetreached == false) {
                    targetreached = true;
                    targetTimer.reset();
                }
            }
            myTimer.reset();
            while(myTimer.milliseconds() < 50) {
                Kicker.setPosition(0);
            }
            if(i == 1) {
                IntakeMotor.setPower(0.7);
                pause(250);
//                IntakeMotor.setPower(0);
//                pause(500);
            }
            else {
                pause(250);
            }
            myTimer.reset();
            while(myTimer.milliseconds() < 250) {
                Kicker.setPosition(0.7);
            }
//            pause(50);
            Kicker.setPosition(0);
        }
        IntakeMotor.setPower(0);
        target = 0;
        while(flywheels.flywheel1.getVelocity() != target && !isStopRequested()) {
            flywheels.setVelocity(0);
        }
//        GreenWheel1.setVelocity(0);
//        GreenWheel2.setVelocity(0);
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

    /** Converts the yaw values obtained from AprilTags to match the orientations used in RoadRunner
     * @return Adjusted Yaw (RoadRunner Orientation)
     */
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
            if(detection.id == 20 || detection.id == 24) {
                return detection;
            }
        }
        // Otherwise we just use whatever tag we find
        return detections.get(0);
    }

    // SUPER EXPERIMENTAL, COMPARE WITH ADJUSTED CAMERA POSE
    // basically uses the angle of the robot and the distance from the camera to the center of the robot (hypotenuse) to find x and y values for roadrunner.
    private Pose2d adjustAprilTagDistance(Pose2d pose) {
        double cameraToCenterInches = 6.0;
        double yaw = currentDetections.get(0).robotPose.getOrientation().getYaw();
        double distanceX = cameraToCenterInches * Math.cos(yaw);
        double distanceY = cameraToCenterInches * Math.sin(yaw);
        return new Pose2d(pose.position.x + distanceX, pose.position.y + distanceY, pose.heading.toDouble());
    }

    private double mean(List<Double> values) {
        double total = 0;
        for (double value: values) {
            total = total + value;
        }
        return total / values.size();
    }

    private void pause(int milliseconds) {
        myTimer.reset();
        while(myTimer.milliseconds() < milliseconds && !isStopRequested()) {

        }
    }
}
