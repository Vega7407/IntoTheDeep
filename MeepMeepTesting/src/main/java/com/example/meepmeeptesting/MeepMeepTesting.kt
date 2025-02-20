package com.example.meepmeeptesting

import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.Vector2d
import com.noahbres.meepmeep.MeepMeep
import com.noahbres.meepmeep.MeepMeep.Background
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder
import kotlin.math.PI

fun main(args: Array<String>) {
    val meepMeep = MeepMeep(800)



    // directions
    val up = PI/2
    val left = PI
    val down = - PI / 2

    // starting positions
    val blueRight = Pose2d(13.0, 61.0, down)
    val blueLeft = Pose2d(-13.0, 61.0, down)
    val redRight = Pose2d(13.0, -61.0, up)
    val redLeft = Pose2d(-13.0, -61.0, up)

    val myBot =
        DefaultBotBuilder(meepMeep) // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
            .setConstraints(60.0, 60.0, Math.toRadians(180.0), Math.toRadians(180.0), 15.0)
            .setDimensions(18.0, 18.0)
            .build()

    myBot.runAction(
        myBot.drive.actionBuilder(redRight)
            .strafeTo(Vector2d(3.2, -34.2))
            .strafeTo(Vector2d(26.3, -34.2))
            .setTangent(up)
            .splineTo(Vector2d(46.5, -9.9), down)
            .strafeTo(Vector2d(46.5, -60.0))
            .strafeTo(Vector2d(46.5, -9.9))
            .strafeTo(Vector2d(55.5, -9.9))
            .strafeTo(Vector2d(55.5, -60.0))
            .strafeTo(Vector2d(55.5, -9.9))
            .strafeTo(Vector2d(62.3 , -9.9))
            .strafeTo(Vector2d(62.3, -62.0))
            .waitSeconds(0.2)
            .strafeTo(Vector2d(2.3, -38.1))
            .strafeTo(Vector2d(3.2, -33.5))
            .waitSeconds(0.2)
            .strafeTo(Vector2d(46.5, -62.0))
            .waitSeconds(0.2)
            .strafeTo(Vector2d(2.3, -38.1))
            .strafeTo(Vector2d(3.2, -33.5))
            .waitSeconds(0.2)
            .strafeTo(Vector2d(46.5, -62.0))
            .build()
    )

    meepMeep.setBackground(Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
        .setDarkMode(true)
        .setBackgroundAlpha(0.95f)
        .addEntity(myBot)
        .start()
}
