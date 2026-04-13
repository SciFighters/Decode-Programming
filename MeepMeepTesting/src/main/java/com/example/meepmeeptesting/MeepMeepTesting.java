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
        MeepMeep meepMeep = new MeepMeep(750);


        RoadRunnerBotEntity wheatley = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(50, 50, Math.PI, Math.PI, 15)
                .setDimensions(18, 18)
                .build();

        TrajectoryActionBuilder wheatleyAuto12 = wheatley.getDrive().actionBuilder(new Pose2d(-41.2, 54.3, 0))
                .setTangent(-Math.PI / 4)
                .splineToConstantHeading(new Vector2d(-22, 24), -Math.PI / 4)

                .setTangent(0)
                .splineToLinearHeading(new Pose2d(14, 44, Math.PI / 2), Math.PI / 2)
                .splineToSplineHeading(new Pose2d(14, 44.1, Math.PI / 2), Math.PI / 2)
                .splineToConstantHeading(new Vector2d(5, 55), Math.PI / 2)
                .setTangent(-Math.PI / 2)//p2
                .splineToConstantHeading(new Vector2d(-13, 20), -Math.PI)

                .setTangent(Math.PI / 2)
                .splineTo(new Vector2d(-13, 52), Math.PI / 2)
                .splineToConstantHeading(new Vector2d(-13, 20), -Math.PI / 2)

                .setTangent(0)
                .splineToConstantHeading(new Vector2d(30, 30), Math.PI / 18 * 6.5)
                .splineToConstantHeading(new Vector2d(36, 48), Math.PI / 2)
                .splineToConstantHeading(new Vector2d(-8, 14), -Math.PI * 3 / 4)

                .splineToConstantHeading(new Vector2d(0, 48), Math.PI / 2);

        TrajectoryActionBuilder wheatleyAuto9 = wheatley.getDrive().actionBuilder(new Pose2d(-47.2, 47.7, Math.PI))
                .setTangent(-Math.PI / 4)
                .splineToLinearHeading(new Pose2d(-30, 30, Math.PI * 3 / 4), -Math.PI / 4)
                .splineToLinearHeading(new Pose2d(-24, 24, Math.PI / 2), -Math.PI / 4)
                .splineToSplineHeading(new Pose2d(-23.9, 23.9, Math.PI / 2), Math.PI / 6)
                .setTangent(Math.PI * 3 / 18)
                .splineToConstantHeading(new Vector2d(-11, 54), Math.PI / 2)
                .splineToSplineHeading(new Pose2d(-11, 54.1, Math.PI / 2), -Math.PI / 2)
                .setTangent(-Math.PI / 2)
                .splineToLinearHeading(new Pose2d(-11, 14, Math.PI * 3 / 4), -Math.PI / 2)
                .setTangent(Math.PI / 6)
                .splineToSplineHeading(new Pose2d(12, 30, Math.PI / 2), Math.PI / 2)
                .splineToConstantHeading(new Vector2d(10, 54), Math.PI / 2)
                .splineToSplineHeading(new Pose2d(10, 54.1, Math.PI / 2), -Math.PI / 2)
                .splineToLinearHeading(new Pose2d(-14, 14, Math.PI * 3 / 4), -Math.PI * 3 / 4)
                .setTangent(Math.PI / 4)
                .splineToLinearHeading(new Pose2d(0, 30, Math.PI / 2), Math.PI / 2);

        TrajectoryActionBuilder wheatleyAuto15 = wheatley.getDrive().actionBuilder(new Pose2d(61.5, 22, Math.PI / 2))
                .splineToConstantHeading(new Vector2d(61.4, 58), Math.PI / 2)
                .splineToConstantHeading(new Vector2d(59, 23), -Math.PI / 2)

                .setTangent(Math.PI)
                .splineToConstantHeading(new Vector2d(9, 26), Math.PI * 3 / 4)
                .splineToConstantHeading(new Vector2d(6, 38), Math.PI * 5 / 8, new TranslationalVelConstraint(20.0))
                .splineToConstantHeading(new Vector2d(5.9, 38.1), Math.PI * 5 / 8)
                .splineToConstantHeading(new Vector2d(2, 53), Math.PI / 2)

                .setTangent(-Math.PI / 2)//p2
                .splineToConstantHeading(new Vector2d(-10, 20), Math.PI)

                .setTangent(Math.PI / 2)
                .splineToConstantHeading(new Vector2d(-12, 46), Math.PI / 2)
                .splineToConstantHeading(new Vector2d(-12, 22), -Math.PI / 2)

                .setTangent(0)
                .splineToConstantHeading(new Vector2d(30, 28), Math.PI / 4)
                .splineToConstantHeading(new Vector2d(36, 52), Math.PI / 2)
                .splineToConstantHeading(new Vector2d(36, 52.1), -Math.PI / 2)

                .splineToConstantHeading(new Vector2d(-8, 16), -Math.PI * 3 / 4)

                .setTangent(Math.PI * 3 / 8)
                .splineToConstantHeading(new Vector2d(0, 48), Math.PI / 2);
        TrajectoryActionBuilder robotReveal = wheatley.getDrive().actionBuilder(new Pose2d(61.5, 22, Math.PI / 2)).
                splineToConstantHeading(new Vector2d(61.4, 58), Math.PI / 2)
                .splineToConstantHeading(new Vector2d(59, 23), -Math.PI / 2)

                .splineToConstantHeading(new Vector2d(61.4, 58), Math.PI / 2)
                .splineToConstantHeading(new Vector2d(59, 23), -Math.PI / 2);
        TrajectoryActionBuilder wheatleyAuto18 = wheatley.getDrive().actionBuilder(new Pose2d(-41.2, 54.3, 0))
                .setTangent(-Math.PI / 4)
                .splineToConstantHeading(new Vector2d(-22, 24), -Math.PI / 4)

                .setTangent(0)
                .splineToLinearHeading(new Pose2d(14, 40,Math.PI/2), Math.PI / 2)
                .splineToSplineHeading(new Pose2d(14, 40.1, Math.PI / 2), Math.PI / 2)
//                .splineToConstantHeading(new Vector2d(14, 48), Math.PI *5/ 8)
                .splineToConstantHeading(new Vector2d(-8,12),-Math.PI*3/4)

                .setTangent(Math.PI / 5)
                .splineToConstantHeading(new Vector2d(4, 30), Math.PI / 3)
                .splineToSplineHeading(new Pose2d(13, 62.5, Math.PI * 11 / 16), Math.PI / 2)
                .setTangent(-Math.PI / 2)//p2
                .splineToConstantHeading(new Vector2d(-6, 14), -Math.PI * 5 / 8)

                .setTangent(Math.PI / 5)
                .splineToConstantHeading(new Vector2d(4, 30), Math.PI / 3)
                .splineToSplineHeading(new Pose2d(13, 62.5, Math.PI * 11 / 16), Math.PI / 2)
                .setTangent(-Math.PI / 2)
                .splineToLinearHeading(new Pose2d(-8, 12, Math.PI / 2), -Math.PI * 3 / 4)

                .setTangent(Math.PI / 5)
                .splineToConstantHeading(new Vector2d(4, 30), Math.PI / 3)
                .splineToSplineHeading(new Pose2d(13, 62.5, Math.PI * 11 / 16), Math.PI / 2)
                .setTangent(-Math.PI/2)//p2
                .splineToLinearHeading(new Pose2d(-13, 20,Math.PI/2),-Math.PI * 3 / 4)

                .setTangent(Math.PI / 2)
                .splineTo(new Vector2d(-13, 52), Math.PI / 2)
                .splineToConstantHeading(new Vector2d(-13, 20), -Math.PI / 2);
        TrajectoryActionBuilder wheatleyAuto18Full = wheatley.getDrive().actionBuilder(new Pose2d(-41.2, 54.3, 0))
                .setTangent(-Math.PI / 4)
                .splineToConstantHeading(new Vector2d(-22, 24), -Math.PI / 4)

                .setTangent(0)
                .splineToLinearHeading(new Pose2d(14, 48,Math.PI/2), Math.PI / 2)
                .splineToSplineHeading(new Pose2d(14, 48.1, Math.PI / 2), Math.PI / 2)
//                .splineToConstantHeading(new Vector2d(14, 48), Math.PI *5/ 8)
                .splineToConstantHeading(new Vector2d(-8,12),-Math.PI*3/4)



                .setTangent(Math.PI / 5)
//                .splineToConstantHeading(new Vector2d(6, 30), Math.PI / 3)
                .splineToLinearHeading(new Pose2d(13, 56, Math.PI * 23 / 36), Math.PI * 3 / 4)
                .setTangent(-Math.PI / 2)//p2
                .splineToLinearHeading(new Pose2d(-8, 12, Math.PI / 2), -Math.PI * 3 / 4)

                .setTangent(Math.PI / 5)
                .splineToConstantHeading(new Vector2d(6, 30), Math.PI / 3)
                .splineToSplineHeading(new Pose2d(16, 62.5, Math.PI * 11 / 16), Math.PI / 2)
                .setTangent(-Math.PI/2)//p2
                .splineToLinearHeading(new Pose2d(-13, 20,Math.PI/2),-Math.PI * 3 / 4)

                .setTangent(Math.PI / 2)
                .splineTo(new Vector2d(-13, 52), Math.PI / 2)
                .splineToConstantHeading(new Vector2d(-13, 20), -Math.PI / 2)

                .setTangent(0)
                .splineToConstantHeading(new Vector2d(30, 28), Math.PI / 4)
                .splineToConstantHeading(new Vector2d(36, 52), Math.PI / 2)
                .splineToConstantHeading(new Vector2d(36, 52.1), -Math.PI / 2)
                .splineToConstantHeading(new Vector2d(-8, 16), -Math.PI * 3 / 4)

                .setTangent(0)
                .splineToConstantHeading(new Vector2d(0, 40), -Math.PI / 2);
        double t = 0.1;
        TrajectoryActionBuilder wheatleyAuto18V2 = wheatley.getDrive().actionBuilder(new Pose2d(-41.2, 54.3, 0))
                .setTangent(-Math.PI / 4)
                .splineToConstantHeading(new Vector2d(-30, 42), -Math.PI / 4)
                .waitSeconds(t)//shoot preload
                //startIntake
                .setTangent(Math.PI / 6)
                .splineToConstantHeading(new Vector2d(-18,47),0)//prepareShooting
                .splineToConstantHeading(new Vector2d(-30,42),-Math.PI * 7 / 8)
                .waitSeconds(t)//shoot
                //start intake
                .setTangent(Math.PI / 9)
                .splineToConstantHeading(new Vector2d(6,47),0)//prepareShooting
                .setTangent(-Math.PI)
                .splineTo(new Vector2d(-14,23),-Math.PI * 26 / 36)
                .waitSeconds(t)//shoot
                //stopped intake
                .setTangent(Math.PI * 10 / 36)
//                .splineTo(new Vector2d(5,40),Math.PI * 18 / 36)
//                .splineToSplineHeading(new Pose2d(5,40.1,Math.PI /2),Math.PI /2)
//                .splineToConstantHeading(new Vector2d(6, 30), Math.PI / 3)
//                .splineToSplineHeading(new Pose2d(16, 62.5, Math.PI * 11 / 16), Math.PI / 2)
                .splineToLinearHeading(new Pose2d(15,62,Math.PI * 10.5 / 16),Math.PI / 2)
                .waitSeconds(t + 1)//intaking artifacts
                .setTangent(-Math.PI*5.5 / 16)
                .splineTo(new Vector2d(-13,20),-Math.PI * 4/5)
                .waitSeconds(t)//shoot
                .setTangent(Math.PI / 5)//start intake
                .splineTo(new Vector2d(30,47),0)
                .setTangent(-Math.PI)
                .splineTo(new Vector2d(-13,20),-Math.PI * 3 / 4)//prepareShooting
                .waitSeconds(t)//shoot
                //start intake
                .setTangent(Math.PI / 4)
                .splineTo(new Vector2d(46,58),0)
                .splineToSplineHeading(new Pose2d(42.1,58,0),-Math.PI)//outtake a bit
                .splineToConstantHeading(new Vector2d(-8,16),-Math.PI * 3 / 4)//prepareShooting
                .waitSeconds(t)//shoot
                .setTangent(Math.PI/4)
                .splineToConstantHeading(new Vector2d(0,30),Math.PI/2)//park
                ;
        double waitTime = 1.5;
        TrajectoryActionBuilder newAuto = wheatley.getDrive().actionBuilder(new Pose2d(-41.2, 54.3, 0))
                .setTangent(-Math.PI / 4)
                .splineToConstantHeading(new Vector2d(-30, 42), -Math.PI / 4)
                .waitSeconds(t)//shoot preload
                //startIntake
                .setTangent(Math.PI / 6)
                .splineToConstantHeading(new Vector2d(-18,47),0)//prepareShooting
                .splineToConstantHeading(new Vector2d(-30,42),-Math.PI * 7 / 8)
                .waitSeconds(t)//shoot
                //start intake
                .setTangent(Math.PI / 9)
                .splineToConstantHeading(new Vector2d(6,47),0)//prepareShooting
                .setTangent(-Math.PI)
                .splineTo(new Vector2d(-14,23),-Math.PI * 26 / 36)
                .waitSeconds(t)//shoot
                //stopped intake
                .setTangent(Math.PI * 10 / 36)
                .splineToLinearHeading(new Pose2d(15,62,Math.PI * 10.5 / 16),Math.PI / 2)
                .waitSeconds(t + 1)//intaking artifacts
                .setTangent(-Math.PI*5.5 / 16)
                .splineToConstantHeading(new Vector2d(-13,20),-Math.PI * 4/5)
                .waitSeconds(t)//shoot
                // stopped intake
                .setTangent(Math.PI * 10 / 36)
                .splineToLinearHeading(new Pose2d(15,62,Math.PI * 10.5 / 16),Math.PI / 2)
                .waitSeconds(t + 1)//intaking artifacts
                .setTangent(-Math.PI*5.5 / 16)
                .splineTo(new Vector2d(-13,20),-Math.PI * 4/5)
                .waitSeconds(t)//shoot
                .setTangent(Math.PI / 5)//start intake
                .splineTo(new Vector2d(30,47),0)
                .setTangent(-Math.PI)
                .splineTo(new Vector2d(-13,20),-Math.PI * 3 / 4)//prepareShooting
                .waitSeconds(t)//shoot
                //start intake
                .setTangent(Math.PI / 4)
                .splineTo(new Vector2d(46,58),0)
                .splineToSplineHeading(new Pose2d(42.1,58,0),-Math.PI)//outtake a bit
                .splineToConstantHeading(new Vector2d(-8,16),-Math.PI * 3 / 4)//prepareShooting
                .waitSeconds(t)//shoot
                .setTangent(Math.PI/4)
                .splineToConstantHeading(new Vector2d(0,30),Math.PI/2);//park

        wheatley.runAction(newAuto.build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
//                .addEntity(needleTestsBot)
                .addEntity(wheatley)
                .start();
    }
}
