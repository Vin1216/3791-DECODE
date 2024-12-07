package org.firstinspires.ftc.teamcode;


import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Disabled
@Autonomous
public class RoadrunnerTest extends LinearOpMode {

    public void runOpMode() {
        Pose2d initialpose = new Pose2d(-72, 36, Math.toRadians(0));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialpose);
        TrajectoryActionBuilder traj1 = drive.actionBuilder(initialpose)
                .splineToSplineHeading(new Pose2d(-66,54,Math.toRadians(95)),Math.toRadians(180));


        waitForStart();
        if(opModeIsActive()) {



//            Pose2d startpose = new Pose2d(0, 0, Math.toRadians(0));



//            Trajectory traj1 = drive.trajectoryBuilder(startpose)
//                    .splineToSplineHeading(new Pose2d(35,35,Math.toRadians(180)),Math.toRadians(0))
//                    .build();

//            Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
//                    .splineToSplineHeading(new Pose2d(35, 35, Math.toRadians(270)),Math.toRadians(0))
//                    .build();

            Action traj1Action = traj1.build();

            Actions.runBlocking(
                    new SequentialAction(
                            traj1Action
                    )
            );
        }
    }
}
