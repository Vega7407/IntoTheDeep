package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
import com.acmerobotics.roadrunner.Vector2d;


public class MeepMeepTesting {
    public static void main(String[] args) {
        double up = Math.PI/2;
        double left = Math.PI;
        double down = - Math.PI / 2;
        MeepMeep meepMeep = new MeepMeep(800);

        Pose2d blueRight = new Pose2d(13.0, 61.0, down);
        Pose2d blueLeft = new Pose2d(-13.0, 61.0, down);
        Pose2d redRight = new Pose2d(9.0, -61.0, down);
        Pose2d redLeft = new Pose2d(-15.0, -61.0, up);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(redLeft)
//                // right side
//                        .setTangent(up)
//                .splineToConstantHeading(new Vector2d(-8, -35), up)
////                        .strafeToConstantHeading(new Vector2d(-8, -38.8))
//                .waitSeconds(1)
//                .setTangent(down)
//                .splineToLinearHeading(new Pose2d(47.1, -44.8, up), 0)
//                .waitSeconds(0.4)
//                .turnTo(down)
//
//                .waitSeconds(0.3)
//                .turnTo((up * 0.67))
//
//                .waitSeconds(0.3)
//                .turnTo(down)
//                .waitSeconds(0.3)
//                // drop and dpad_right or up to wall
//
//                .strafeTo(new Vector2d(49, -67))
//                .waitSeconds(0.2)
//                        .setTangent(up)
//                .splineToConstantHeading(new Vector2d(-5, -30), up)
//                .waitSeconds(0.2)
//                .setTangent(down)
//                .splineToConstantHeading(new Vector2d(45, -69), down)
//                .waitSeconds(0.2)
//                .setTangent(up)
//                .splineToConstantHeading(new Vector2d(0, -30), up)
//                .waitSeconds(0.2)
//                .setTangent(down)
//                .splineToConstantHeading(new Vector2d(45, -67), down)
//                .waitSeconds(0.2)
//                .setTangent(up)
//                .splineToConstantHeading(new Vector2d(6, -25), up)


//                left side
                //13.31 s strafeToSplineHeading
                //
                // go to basket
                        .setTangent(Math.PI)
                .splineToLinearHeading(new Pose2d(-57.6, -52.2, up*0.5), Math.PI )
                .waitSeconds(0.5)
                // go to right sample
                        .setTangent(up)
                .splineToLinearHeading(new Pose2d(-49.5, -42, up), up)
                .waitSeconds(0.5)
                // go to basket
                        .setTangent(Math.PI)
                .strafeToLinearHeading(new Vector2d(-56.5, -50), up * 0.5)
                .waitSeconds(0.5)
                // go to middle sample
                .strafeToSplineHeading(new Vector2d(-58.5, -41.5), up)
                .waitSeconds(0.5)
                // go to basket
                .strafeToSplineHeading(new Vector2d(-56, -48), up * 0.5)
                .waitSeconds(0.5)
                // go to left sample
                .strafeToSplineHeading(new Vector2d(-57.6, -37.0), up * 1.4)
                .waitSeconds(0.5)
                // go to basket
                .strafeToSplineHeading(new Vector2d(-58, -52.2), up * 0.5)
                .waitSeconds(0.5)
                // level 1 ascent
                .strafeToSplineHeading(new Vector2d(-24.1, -4.2), -Math.PI)
//
//                .strafeTo(new Vector2d(3.2, -34.2))
//                .strafeTo(new Vector2d(26.3, -34.2))
//                .setTangent(up)
//                .splineTo(new Vector2d(46.5, -9.9), down)
//                .strafeTo(new Vector2d(46.5, -60.0))
//                .strafeTo(new Vector2d(46.5, -9.9))
//                .strafeTo(new Vector2d(55.5, -9.9))
//                .strafeTo(new Vector2d(55.5, -60.0))
//                .strafeTo(new Vector2d(55.5, -9.9))
//                .strafeTo(new Vector2d(62.3 , -9.9))
//                .strafeTo(new Vector2d(62.3, -62.0))
//                .waitSeconds(0.2)
//                .strafeTo(new Vector2d(2.3, -38.1))
//                .strafeTo(new Vector2d(3.2, -33.5))
//                .waitSeconds(0.2)
//                .strafeTo(new Vector2d(46.5, -62.0))
//                .waitSeconds(0.2)
//                .strafeTo(new Vector2d(2.3, -38.1))
//                .strafeTo(new Vector2d(3.2, -33.5))
//                .waitSeconds(0.2)
//                .strafeTo(new Vector2d(46.5, -62.0))
                .build());
                /*.lineToX(30)
                .turn(Math.toRadians(90))
                .lineToY(30)
                .turn(Math.toRadians(90))
                .lineToX(0)
                .turn(Math.toRadians(90))
                .lineToY(0)
                .turn(Math.toRadians(90))
                .build());*/

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK  )
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}