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
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 19.544)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-33, -62, Math.toRadians(0)))
                //spline to bucket
                .splineToLinearHeading(new Pose2d(-55, -55, Math.toRadians(45)), -pi/8)
                .waitSeconds(1)

                //code to drop off sample (top bucket)


                //face first sample
                .splineToLinearHeading(new Pose2d(-55, -55, Math.toRadians(76.79)), -pi/8)
                .waitSeconds(1)

                //grab


                //rotate to bucket
                .splineToLinearHeading(new Pose2d(-55, -55, Math.toRadians(45)), -pi/8)
                .waitSeconds(1)

                //drop sample


                //face second sample
                .splineToLinearHeading(new Pose2d(-55, -55, Math.toRadians(93.98)), -pi/8)
                .waitSeconds(1)

                //grab


                //rotate to bucket
                .splineToLinearHeading(new Pose2d(-55, -55, Math.toRadians(45)), -pi/8)
                .waitSeconds(1)

                //drop sample


                //face third sample
                .splineToLinearHeading(new Pose2d(-55, -55, Math.toRadians(111.345)), -pi/8)
                .waitSeconds(1)

                //grab


                //face bucket
                .splineToLinearHeading(new Pose2d(-55, -55, Math.toRadians(45)), -pi/8)

                //drop sample


                //park
                .splineToLinearHeading(new Pose2d(-55, -55, Math.toRadians(100)), -pi/8)
                .splineTo(new Vector2d(-25, -11.5), 0)

                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_OFFICIAL)
                .setDarkMode(false)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}