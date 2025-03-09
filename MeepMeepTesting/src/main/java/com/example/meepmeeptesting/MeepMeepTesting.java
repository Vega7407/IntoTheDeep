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
        Pose2d redLeft = new Pose2d(-13.0, -61.0, up);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(redRight)
                .strafeToConstantHeading(new Vector2d(1.2, -32.8))
                .waitSeconds(1)
                .setTangent(down)
                .splineTo(new Vector2d(51.2, -47.5), up)
                .turnTo(down)
                .turnTo((up * 0.8))
                .turnTo(down)
                .strafeTo(new Vector2d(51, -65))
                .waitSeconds(0.1)
                .setTangent(down)

                .strafeToConstantHeading(new Vector2d(1.2, -35.2))
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