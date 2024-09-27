package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.List;

@Autonomous
public class ItDLeftAuto extends LinearOpMode {
    AprilTagProcessor myAprilTagProcessor;
    AprilTagDetection myAprilTagDetection;
    List<AprilTagDetection> myAprilTagDetections;
    double myAprilTagPoseX = 0;
    double myAprilTagPoseBearing;
    double myAprilTagPoseRange;
    double myAprilTagPoseYaw;
    Integer myAprilTagIdCode;

    ElapsedTime AutoPeriodTime = new ElapsedTime();

    String VincentsMessage = "My job is done. You guys better score a million points in TeleOp - Vincent";

    int AllianceColor = 1; // 1 for blue, -1 for red, will change based on AprilTags
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Init_VisionPortal();
        DetectAprilTags();
        // TODO: I'm not sure if I should stop the program automatically if it can't find an AprilTag
//        if(myAprilTagDetection.metadata == null) {
//            telemetry.addLine("No AprilTag Detected!");
//            telemetry.addLine("Please restart the program");
//            telemetry.update();
////            requestOpModeStop();
//        }
//        // Change Alliance color to -1 (red) if the AprilTag on the blue side is detected (id 12)
//        // Now we can just multiply all the positions by this number to reflect it over the origin!
//        else if (myAprilTagDetection.metadata.id == 12) {
//            AllianceColor = -1;
//        }
        // startpose will be reflected over the origin if the alliance color is red
        // However, the angles will stay relative to the robot, since it's easier to think of it "turning" those degrees.
        Pose2d startpose = new Pose2d(-72 * AllianceColor, 36 * AllianceColor, Math.toRadians(0));
        // Change starting pose estimate based on the opposite AprilTag's x (horizontal) value
        startpose = new Pose2d(startpose.getX(), startpose.getY() + myAprilTagPoseX, Math.toRadians(0));
        drive.setPoseEstimate(startpose);
        telemetry.addData("PoseEstimate",startpose);
        telemetry.update();

        Trajectory PreloadTraj = drive.trajectoryBuilder(startpose)
                .splineToSplineHeading(new Pose2d(-66 * AllianceColor,54 * AllianceColor,Math.toRadians(135)),Math.toRadians(135))
                .build();

        Trajectory TopSampleTraj = drive.trajectoryBuilder(PreloadTraj.end())
                .splineToSplineHeading(new Pose2d(-24 * AllianceColor,54 * AllianceColor,Math.toRadians(0)),Math.toRadians(0))
                .build();

        // This is so annoying, but I don't want to make trajectories on the fly since that causes a delay
        Trajectory TopSampleScoreTraj = drive.trajectoryBuilder(TopSampleTraj.end())
                .splineToSplineHeading(new Pose2d(-66 * AllianceColor, 54 * AllianceColor, Math.toRadians(135)),Math.toRadians(135))
                .build();

        Trajectory MiddleSampleTraj = drive.trajectoryBuilder(PreloadTraj.end())
                .splineToSplineHeading(new Pose2d(-24 * AllianceColor,48 * AllianceColor,Math.toRadians(0)),Math.toRadians(0))
                .build();

        Trajectory MiddleSampleScoreTraj = drive.trajectoryBuilder(MiddleSampleTraj.end())
                .splineToSplineHeading(new Pose2d(-66 * AllianceColor, 54 * AllianceColor, Math.toRadians(135)),Math.toRadians(135))
                .build();

        Trajectory BottomSampleTraj = drive.trajectoryBuilder(PreloadTraj.end())
                .splineToSplineHeading(new Pose2d(-24 * AllianceColor,42 * AllianceColor,Math.toRadians(0)),Math.toRadians(0))
                .build();

        Trajectory BottomSampleScoreTraj = drive.trajectoryBuilder(BottomSampleTraj.end())
                .splineToSplineHeading(new Pose2d(-66 * AllianceColor, 54 * AllianceColor, Math.toRadians(135)),Math.toRadians(135))
                .build();

        // I'm not entirely sure if this start pose will work, but they should all end up here right???
        Trajectory ParkTraj = drive.trajectoryBuilder(new Pose2d(-66 * AllianceColor,42 * AllianceColor,Math.toRadians(135)))
                .splineTo(new Vector2d(0 * AllianceColor,24 * AllianceColor), Math.toRadians(270))
                .build();


        String state = "ScorePreload";

        // Create a list of the possible states where the samples are. This is to keep track of which states have been used already.
        List<String> PickupSampleStates = new ArrayList<>();
        PickupSampleStates.add("PickupTopSample");
        PickupSampleStates.add("PickupMiddleSample");
        PickupSampleStates.add("PickupBottomSample");
        waitForStart();
        if(opModeIsActive()) {
            AutoPeriodTime.reset();
            while(opModeIsActive()) {
                switch(state) {
                    case "ScorePreload":
                        drive.followTrajectory(PreloadTraj);
                        // TODO: Add scoring motor and servo movements here
                        state = "PickupTopSample";
                        break;
                    case "PickupTopSample":
                        drive.followTrajectory(TopSampleTraj);
                        // TODO: Add intake movements here
                        PickupSampleStates.remove("PickupTopSample");
                        state = "Score";
                        break;
                    case "PickupMiddleSample":
                        drive.followTrajectory(MiddleSampleTraj);
                        // TODO: Add intake movements here
                        PickupSampleStates.remove("PickupMiddleSample");
                        state = "Score";
                        break;
                    case "PickupBottomSample":
                        drive.followTrajectory(BottomSampleTraj);
                        // TODO: Add intake movements here
                        PickupSampleStates.remove("PickupBottomSample");
                        state = "Score";
                        break;
                    case "Score":
                        // Maybe this can be turned into a hashmap or another switch case?
                        if(drive.getPoseEstimate().getY() >= 53) {
                            drive.followTrajectory(TopSampleScoreTraj);
                        }
                        else if(drive.getPoseEstimate().getY() <= 42) {
                            drive.followTrajectory(BottomSampleScoreTraj);
                        }
                        else {
                            drive.followTrajectory(MiddleSampleScoreTraj);
                        }
                        // TODO: Add scoring movements here (maybe this can be its own case)

                        // Goes park if there are no more samples to collect
                        if(PickupSampleStates.isEmpty()) {
                            state = "Park";
                        }
                        else {
                            state = PickupSampleStates.get(0);
                        }
                        break;
                    case "Park":
                        drive.followTrajectory(ParkTraj);
                        state = VincentsMessage;
                        break;
                }

                // If time is running out for the autonomous period, immediately go park.
                // TODO: Adjust time limit based on how much time is needed to park.
                if(AutoPeriodTime.seconds() >= 25 && state != VincentsMessage) {
                    state = "Park";
                }
                telemetry.addData("Current State: ",state);
                telemetry.update();
            }
        }
    }

    private void Init_VisionPortal() {
        VisionPortal myVisionPortal;
        // Create the AprilTag processor and assign it to a variable.
        myAprilTagProcessor = AprilTagProcessor.easyCreateWithDefaults();
        // Create a VisionPortal, with the specified webcam name, AprilTag processor,
        // and assign it to a variable.
        myVisionPortal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "WebCam"), myAprilTagProcessor);
    }

    private void DetectAprilTags() {
        // Get a list containing the latest detections, which may be stale.
        myAprilTagDetections = myAprilTagProcessor.getDetections();
        for (AprilTagDetection myAprilTagDetection_item : myAprilTagDetections) {
            myAprilTagDetection = myAprilTagDetection_item;
            if (myAprilTagDetection.metadata != null) {
                myAprilTagIdCode = myAprilTagDetection.id;
                myAprilTagPoseX = myAprilTagDetection.ftcPose.x;
                myAprilTagPoseBearing = myAprilTagDetection.ftcPose.bearing;
                myAprilTagPoseRange = myAprilTagDetection.ftcPose.range;
                myAprilTagPoseYaw = myAprilTagDetection.ftcPose.yaw;
            }
        }
    }
}
