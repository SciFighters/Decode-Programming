package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) { //-70,45.6 ,44.6, 64.5
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity clankerTestsBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.PI, Math.PI, 15)
                .setDimensions(17.5,15.5)
                .build();

        TrajectoryActionBuilder normalAuto = clankerTestsBot.getDrive().actionBuilder(new Pose2d(-38,56,0))
                .setTangent(-Math.PI / 4)
                .splineToLinearHeading(new Pose2d(-24,36,Math.PI / 4),-Math.PI/4)
                .waitSeconds(1)
//                .waitSeconds(0.5)
                .setTangent(Math.PI / 4)
                .splineToSplineHeading(new Pose2d(-12,48,Math.PI/2),Math.PI/2)
                .splineToConstantHeading(new Vector2d(-6,55),Math.PI/4)
                .setTangent(Math.PI * 1.5)
                .splineTo(new Vector2d(-18,26),Math.PI* 10/9).waitSeconds(1.5)
                .setTangent(Math.PI / 9)
                .splineTo(new Vector2d(12,54),Math.PI/2)
                .setTangent(-Math.PI/2)
                .splineTo(new Vector2d(-12,20),Math.PI).waitSeconds(1.5)
                .setTangent(0)
                .splineTo(new Vector2d(36,54),Math.PI/2)
                .splineToSplineHeading(new Pose2d(36,56,Math.PI/2),Math.PI/2)
                .splineToConstantHeading(new Vector2d(56,16),-Math.PI/2).waitSeconds(1);

        clankerTestsBot.runAction(normalAuto.build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
//                .addEntity(needleTestsBot)
                .addEntity(clankerTestsBot)
                .start();
    }
}