package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(100, 100, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(66, -12, Math.toRadians(180)))
//                .splineToSplineHeading(new Pose2d(60,-12,Math.toRadians(225)),Math.toRadians(225))
                        .lineToX(60)
                .waitSeconds(1)
//                .turnTo(Math.toRadians(180))
//                .setReversed(true)
                .splineToLinearHeading(new Pose2d(36,-30,Math.toRadians(90)),Math.toRadians(270))
                .lineToY(-48)
                .waitSeconds(1)
                .splineToLinearHeading(new Pose2d(66,-12,Math.toRadians(225)),Math.toRadians(0))
                        .waitSeconds(1)
//                        .setReversed(false)
//                .turnTo(Math.toRadians(90))
//                        .splineToLinearHeading(new Pose2d(12,-12,Math.toRadians(90)),Math.toRadians(180))
//                        .strafeTo(new Vector2d(0,-12))
                        .splineToLinearHeading(new Pose2d(12,-30,Math.toRadians(90)),Math.toRadians(270))
                                .lineToY(-48)
                .splineToLinearHeading(new Pose2d(66,-12,Math.toRadians(225)),Math.toRadians(0))
//                .splineToSplineHeading(new Pose2d(12,-36,Math.toRadians(90)),Math.toRadians(270))
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}