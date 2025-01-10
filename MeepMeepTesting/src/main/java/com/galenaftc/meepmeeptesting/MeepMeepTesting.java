package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        double pi = Math.PI;
        MeepMeep meepMeep = new MeepMeep(900);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-33, -62, Math.toRadians(0)))
//                .setTangent(45)
                .splineToLinearHeading(new Pose2d(-55, -55, Math.toRadians(45)), -pi/8)
                .waitSeconds(1)
//                .splineToLinearHeading((new Pose2d(-55, -55, Math.toRadians(76.79169703))), 0)
                        .turn(Math.toRadians(31.79169703))
                .waitSeconds(1)
//
                .turn(Math.toRadians(17.18630621))
                .waitSeconds(1)
                .turn(Math.toRadians(17.36528606))
                .waitSeconds(1)
                .splineToLinearHeading(new Pose2d(-55, -55, 0), 0)
                .lineToX(62)
                .linetoY(62)
                .waitSeconds(1)


                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_OFFICIAL)
                .setDarkMode(false)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}