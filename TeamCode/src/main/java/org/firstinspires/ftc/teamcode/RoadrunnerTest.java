package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous
public class RoadrunnerTest extends LinearOpMode {

    public void runOpMode() {
        waitForStart();
        if(opModeIsActive()) {
            SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

            Pose2d startpose = new Pose2d(0, 0, Math.toRadians(0));

            drive.setPoseEstimate(startpose);

            Trajectory traj1 = drive.trajectoryBuilder(startpose)
                    .splineToSplineHeading(new Pose2d(35,0,Math.toRadians(180)),Math.toRadians(180))
                    .build();

            Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                    .splineToSplineHeading(new Pose2d(35, 35, Math.toRadians(270)),Math.toRadians(270))
                    .build();

            drive.followTrajectory(traj1);
            drive.followTrajectory(traj2);

        }
    }
}
