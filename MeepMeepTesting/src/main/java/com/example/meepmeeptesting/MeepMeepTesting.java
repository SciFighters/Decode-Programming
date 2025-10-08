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
                .setDimensions(18, 18)
                .build();

        TrajectoryActionBuilder normalAuto = clankerTestsBot.getDrive().actionBuilder(new Pose2d(-38, 56, 0))
                .setTangent(-Math.PI / 4)
                .splineToLinearHeading(new Pose2d(-22, 34, Math.PI / 4), -Math.PI / 4)
//                .waitSeconds(1)
//                .waitSeconds(0.5)
                .setTangent(Math.PI / 4)
                .splineToSplineHeading(new Pose2d(-12, 48, Math.PI / 2), Math.PI / 2)
                .splineToConstantHeading(new Vector2d(-6, 55), Math.PI / 4)
                .setTangent(Math.PI * 1.5)
                .splineTo(new Vector2d(-18, 26), Math.PI * 10 / 9)
//                .waitSeconds(1.5)
                .setTangent(Math.PI / 9)
                .splineTo(new Vector2d(12, 54), Math.PI / 2)
                .setTangent(-Math.PI / 2)
                .splineTo(new Vector2d(-12, 20), Math.PI)
//                .waitSeconds(1.5)
                .setTangent(0)
                .splineTo(new Vector2d(36, 54), Math.PI / 2)
                .splineToSplineHeading(new Pose2d(36, 56, Math.PI / 2), Math.PI / 2)
                .splineToConstantHeading(new Vector2d(56, 16), -Math.PI / 2);
        TrajectoryActionBuilder betterAuto = clankerTestsBot.getDrive().actionBuilder(new Pose2d(-38, 56, 0))
                .setTangent(-Math.PI / 4)
                .splineToLinearHeading(new Pose2d(-22, 34, Math.PI / 4), -Math.PI / 4)
//                .waitSeconds(1)
//                .waitSeconds(0.5)
                .setTangent(Math.PI / 4)
                .splineTo(new Vector2d(-12, 50), Math.PI / 2)
//                .splineToConstantHeading(new Vector2d(-6,55),Math.PI/4)
                .setTangent(Math.PI * 1.5)
                .splineTo(new Vector2d(-18, 26), Math.PI * 10 / 9)
//                .waitSeconds(1.5)
                .setTangent(Math.PI / 9)
                .splineTo(new Vector2d(12, 48), Math.PI / 2)
                .splineToSplineHeading(new Pose2d(12, 50, Math.PI / 2), Math.PI / 2)
                .splineToConstantHeading(new Vector2d(5, 55), Math.PI * 3 / 4)
                .setTangent(-Math.PI / 2)
                .splineTo(new Vector2d(-12, 20), Math.PI)
//                .waitSeconds(1.5)
                .setTangent(0)
                .splineTo(new Vector2d(36, 54), Math.PI / 2)
                .splineToSplineHeading(new Pose2d(36, 56, Math.PI / 2), Math.PI / 2)
                .splineToConstantHeading(new Vector2d(56, 16), -Math.PI / 2);
        TrajectoryActionBuilder showcase = clankerTestsBot.getDrive().actionBuilder(new Pose2d(0, 36, -Math.PI))
                .setTangent(0)
                .splineToLinearHeading(new Pose2d(38, -33, 0), 0);
        TrajectoryActionBuilder theBest = clankerTestsBot.getDrive().actionBuilder((new Pose2d(61, 23.0 + 1.0 / 3, Math.PI)))
                .splineToSplineHeading(new Pose2d(52, 23.0, Math.PI / 2), Math.PI)
                .setTangent(Math.PI / 2)
                .splineToConstantHeading(new Vector2d(61, 61), Math.PI / 2)
//                .setTangent(-Math.PI/2)
                .splineToConstantHeading(new Vector2d(56, 48), -Math.PI * 3 / 5)
                .splineToSplineHeading(new Pose2d(54, 8, Math.PI * 3 / 4), -Math.PI / 4)
                .splineToSplineHeading(new Pose2d(54.1, 7.9, Math.PI * 3 / 4), Math.PI * 3 / 4)
//                .setTangent(Math.PI * 3/4)
                .splineTo(new Vector2d(9, 50), Math.PI / 2)
                .splineToSplineHeading(new Pose2d(9, 50.1, Math.PI / 2), Math.PI / 2)
                .splineToConstantHeading(new Vector2d(-9, 9), -Math.PI * 2 / 3)
                .splineToConstantHeading(new Vector2d(-11, 46), Math.PI / 2)
                .splineToSplineHeading(new Pose2d(-11, 46.1, Math.PI / 2), -Math.PI / 2)
                .splineTo(new Vector2d(-16, 15), -Math.PI * 7 / 9)
                .splineToSplineHeading(new Pose2d(-16.076604444311, 14.93572123903, Math.PI * 2 / 9), Math.PI * 2 / 9)
//                .setTangent(Math.PI * 2 / 9)
                .splineTo(new Vector2d(35.5, 47), 0)
                .splineTo(new Vector2d(58, 20), -Math.PI / 2);


        clankerTestsBot.runAction(theBest.build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
//                .addEntity(needleTestsBot)
                .addEntity(clankerTestsBot)
                .start();
    }
}
