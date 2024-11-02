package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.List;

@Autonomous
public class ItDLeftAuto extends LinearOpMode {

    //TODO: Fix this when I understand how RR Actions work

//    public class Army {
//        private DcMotorEx Army;
//
//        public Army(HardwareMap hardwareMap) {
//            Army = hardwareMap.get(DcMotorEx.class, "ArmMotor");
//            Army.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//            Army.setDirection(DcMotorSimple.Direction.FORWARD);
//        }
//
//        public class ArmyUp implements Action {
//            private boolean initialized = false;
//
//            @Override
//            public boolean run(@NonNull TelemetryPacket packet) {
//                if (!initialized) {
//                    Army.setPower(-0.9);
//                    initialized = true;
//                }
//
//                double pos = Army.getCurrentPosition();
//                packet.put("ArmPos",pos);
//                if (pos > 0) {
//                    return true;
//                }
//                else {
//                    Army.setPower(0);
//                    return false;
//                }
//            }
//        }
//        public Action armyUp() {
//            return new ArmyUp();
//        }
//
//        public class ArmyDown implements Action {
//            private boolean initialized = false;
//
//            @Override
//            public boolean run(@NonNull TelemetryPacket packet) {
//                if (!initialized) {
//                    Army.setPower(0.9);
//                    initialized = true;
//                }
//
//                double pos = Army.getCurrentPosition();
//                packet.put("ArmPos", pos);
//                if (pos < 1300) {
//                    return true;
//                } else {
//                    Army.setPower(0);
//                    return false;
//                }
//            }
//        }
//        public Action armyDown() {
//            return new ArmyDown();
//        }
//    }
    DcMotorEx Arm;
    DcMotorEx Lift;
    DcMotorEx Intake;

    Servo Bucket;

    AprilTagProcessor myAprilTagProcessor;
    AprilTagDetection myAprilTagDetection;
    List<AprilTagDetection> myAprilTagDetections;
    double myAprilTagPoseX = 0;
    double myAprilTagPoseBearing;
    double myAprilTagPoseRange;
    double myAprilTagPoseYaw;
    Integer myAprilTagIdCode;

    ElapsedTime AutoPeriodTime;
    ElapsedTime myTimer;

    int ticksperRevolution = 550;
    double wheelCircumference = 12.56;
    double ticksPerInch = ticksperRevolution / wheelCircumference;
    int tickstoDestination;
    static final double INCREMENT = 0.01;     // amount to ramp motor each CYCLE_MS cycle
    static final int CYCLE_MS = 50;     // period of each cycle
    static final double MAX_FWD = 0.75;     // Maximum FWD power applied to motor
    static final double MAX_REV = -0.75;     // Maximum REV power applied to motor
    boolean rampUp = true;
    double power;
    DcMotorEx FrontLeft;
    DcMotorEx FrontRight;
    DcMotorEx RearLeft;
    DcMotorEx RearRight;

    String VincentsMessage = "My job is done. You guys better score a million points in TeleOp - Vincent";

    public void runOpMode() {
        Pose2d startpose = new Pose2d(-72, 36, Math.toRadians(0));
        MecanumDrive drive = new MecanumDrive(hardwareMap, startpose);

        FrontLeft = drive.leftFront;
        FrontRight = drive.rightFront;
        RearLeft = drive.leftBack;
        RearRight = drive.rightBack;

        Arm = hardwareMap.get(DcMotorEx.class, "ArmMotor");
        Lift = hardwareMap.get(DcMotorEx.class,"LiftMotor");
        Intake = hardwareMap.get(DcMotorEx.class,"IntakeMotor");

        Bucket = hardwareMap.get(Servo.class,"bucket");

        AutoPeriodTime = new ElapsedTime();
        myTimer = new ElapsedTime();

        Lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        DetectAprilTags();
        // TODO: I'm not sure if I should stop the program automatically if it can't find an AprilTag
//        if(myAprilTagDetection.metadata == null) {
//            telemetry.addLine("No AprilTag Detected!");
//            telemetry.addLine("Please restart the program");
//            telemetry.update();
////            requestOpModeStop();
//        }
        // startpose will be reflected over the origin if the alliance color is red
        // However, the angles will stay relative to the robot, since it's easier to think of it "turning" those degrees.

        // Change starting pose estimate based on the opposite AprilTag's x (horizontal) value
//        startpose = new Pose2d(startpose.position.x, startpose.position.y + myAprilTagPoseX, Math.toRadians(0));
        telemetry.addData("PoseEstimate",startpose);
        telemetry.update();

        TrajectoryActionBuilder PreloadTraj = drive.actionBuilder(startpose)
                .splineToSplineHeading(new Pose2d(-66,54,Math.toRadians(95)),Math.toRadians(180));

        TrajectoryActionBuilder TopSampleTraj = PreloadTraj.fresh()
                .splineToSplineHeading(new Pose2d(-24,48,Math.toRadians(270)),Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(-24,54, Math.toRadians(270)),Math.toRadians(90));

        TrajectoryActionBuilder MiddleSampleTraj = PreloadTraj.fresh()
                .splineToSplineHeading(new Pose2d(-24,42,Math.toRadians(270)),Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(-24,48,Math.toRadians(270)),Math.toRadians(90));

        TrajectoryActionBuilder BottomSampleTraj = PreloadTraj.fresh()
//                .turnTo(Math.toRadians(270))
//                .lineToY(36)
//                .strafeTo(new Vector2d(-24, 36));
                .splineToSplineHeading(new Pose2d(-24,36,Math.toRadians(270)),Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(-24,42,Math.toRadians(270)),Math.toRadians(90));



        String state = "ScorePreload";

        // Create a list of the possible states where the samples are. This is to keep track of which states have been used already.
        List<String> PickupSampleStates = new ArrayList<>();
        PickupSampleStates.add("PickupBottomSample");
//        PickupSampleStates.add("PickupMiddleSample");
//        PickupSampleStates.add("PickupTopSample");
        //Middle and top are disabled for now since the systems do not move fast enough

        Bucket.setPosition(1);

        telemetry.addLine("Initialization Complete");
        telemetry.update();

        waitForStart();
        if(opModeIsActive()) {
            AutoPeriodTime.reset();
            while(opModeIsActive()) {
                switch(state) {
                    case "Test":
//                        EncoderMoveToPosition(Lift,3975,0.6);
                        ScoreOnHighBasket();
                        state = "AAAAAAAAAAAAAAAAAAA";
                        break;
                    case "ScorePreload":
                        Actions.runBlocking(new SequentialAction(PreloadTraj.build()));
                        sleep(1000);
                        idle();
                        ScoreOnHighBasket();
                        state = "A";
                        break;
                    case "PickupTopSample":
                        Actions.runBlocking(new SequentialAction(TopSampleTraj.build()));
                        PickupSample();
                        PickupSampleStates.remove("PickupTopSample");
                        state = "Score";
                        break;
                    case "PickupMiddleSample":
                        Actions.runBlocking(new SequentialAction(MiddleSampleTraj.build()));
                        PickupSample();
                        PickupSampleStates.remove("PickupMiddleSample");
                        state = "Score";
                        break;
                    case "PickupBottomSample":
                        Actions.runBlocking(new SequentialAction(BottomSampleTraj.build()));
                        PickupSample();
                        PickupSampleStates.remove("PickupBottomSample");
                        state = "Score";
                        break;
                    case "Score":
                        // Maybe this can be turned into a hashmap or another switch case?
//                        if(drive.getPoseEstimate().getY() >= 66) {
//                            drive.followTrajectory(TopSampleScoreTraj);
//                        }
//                        else if(drive.getPoseEstimate().getY() <= 54) {
//                            drive.followTrajectory(BottomSampleScoreTraj);
//                        }
//                        else {
//                            drive.followTrajectory(MiddleSampleScoreTraj);
//                        }
                        TrajectoryActionBuilder Score = drive.actionBuilder(drive.pose)
                                .splineToSplineHeading(new Pose2d(-66,54,Math.toRadians(95)),Math.toRadians(180));

                        Actions.runBlocking(new SequentialAction(Score.build()));

                        ScoreOnHighBasket();

                        // Goes park if there are no more samples to collect
                        if(PickupSampleStates.isEmpty()) {
                            state = "Park";
                        }
                        else {
                            state = "PickupBottomSample";
                        }
                        break;
                    case "Park":
                        TrajectoryActionBuilder ParkTraj = drive.actionBuilder(drive.pose)
                                .splineTo(new Vector2d(0,24), Math.toRadians(270));
                        Actions.runBlocking(new SequentialAction(ParkTraj.build()));
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

    private void PickupSample() {
        while(myTimer.milliseconds() < 200) {
            Arm.setPower(0.9);
        }
        Arm.setPower(0);
        Intake.setPower(-1);
        sleep(500);
        idle();
        Intake.setPower(0);
        while(myTimer.milliseconds() < 200) {
            Arm.setPower(0.9);
        }
        Arm.setPower(0);
        Intake.setPower(-1);
        sleep(500);
        idle();
        Intake.setPower(0);
    }

    private void ScoreOnHighBasket() {
        EncoderMoveToPosition(Arm,1300,0.9);
        Bucket.setPosition(1);
        EncoderMoveToPosition(Lift,3975,0.6); //This function is being skipped for some reason
        Bucket.setPosition(0.3);
        sleep(1000);
        idle();
        Bucket.setPosition(1);
        EncoderMoveToPosition(Lift,0,0.6);
        EncoderMoveToPosition(Arm,0,0.9);
        sleep(500);
        idle();
    }

//    private void Init_VisionPortal() {
//        VisionPortal myVisionPortal;
//        // Create the AprilTag processor and assign it to a variable.
//        myAprilTagProcessor = AprilTagProcessor.easyCreateWithDefaults();
//        // Create a VisionPortal, with the specified webcam name, AprilTag processor,
//        // and assign it to a variable.
//        myVisionPortal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "WebCam"), myAprilTagProcessor);
//    }

//    private void DetectAprilTags() {
//        // Get a list containing the latest detections, which may be stale.
//        myAprilTagDetections = myAprilTagProcessor.getDetections();
//        for (AprilTagDetection myAprilTagDetection_item : myAprilTagDetections) {
//            myAprilTagDetection = myAprilTagDetection_item;
//            if (myAprilTagDetection.metadata != null) {
//                myAprilTagIdCode = myAprilTagDetection.id;
//                myAprilTagPoseX = myAprilTagDetection.ftcPose.x;
//                myAprilTagPoseBearing = myAprilTagDetection.ftcPose.bearing;
//                myAprilTagPoseRange = myAprilTagDetection.ftcPose.range;
//                myAprilTagPoseYaw = myAprilTagDetection.ftcPose.yaw;
//            }
//        }
//    }

    private void EncoderMoveToPosition(DcMotorEx motor, int ticks, double power) {
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setTargetPosition(ticks);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        if(ticks > motor.getCurrentPosition()) {
            motor.setPower(power);
        }
        else {
            motor.setPower(-power);
        }
        while(motor.isBusy()) {
            telemetry.addData("motor ticks ", motor.getCurrentPosition());
            telemetry.update();
        }
        motor.setPower(0);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private void MoveForwardEncoder(int Distance) {
        ResetEncoder();
        tickstoDestination = (int) (Distance * ticksPerInch);
        FrontLeft.setTargetPosition(tickstoDestination);
        FrontRight.setTargetPosition(tickstoDestination);
        RearLeft.setTargetPosition(tickstoDestination);
        RearRight.setTargetPosition(tickstoDestination);
        FrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RearLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RearRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FrontLeft.setPower(0.1);
        FrontRight.setPower(0.1);
        RearLeft.setPower(0.1);
        RearRight.setPower(0.1);
        power = 0.1;
        while (FrontRight.isBusy()) {
            if (rampUp) {
                // Keep stepping up until we hit the max value.
                power += INCREMENT;
                if (power >= MAX_FWD) {
                    power = MAX_FWD;
                    rampUp = !rampUp;   // Switch ramp direction
                }
            } else {
                // Keep stepping down until we hit the min value.
                power -= INCREMENT;
                if (power <= MAX_REV) {
                    power = MAX_REV;
                    rampUp = !rampUp;  // Switch ramp direction
                }
            }
            FrontLeft.setPower(power);
            FrontRight.setPower(power);
            RearLeft.setPower(power);
            RearRight.setPower(power);
            myTimer.reset();
            while (myTimer.milliseconds() <= CYCLE_MS) {
                telemetry.update();
            }
        }
        FrontLeft.setPower(0);
        FrontRight.setPower(0);
        RearLeft.setPower(0);
        RearRight.setPower(0);
        DisableEncoders();
    }
    private void DisableEncoders() {
        FrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RearLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RearRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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
}
