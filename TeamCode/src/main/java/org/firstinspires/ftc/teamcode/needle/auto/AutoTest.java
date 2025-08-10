package org.firstinspires.ftc.teamcode.needle.auto;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.MecanumDrive;


@Autonomous
public class AutoTest extends LinearOpMode {
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();


    @Override
    public void runOpMode() throws InterruptedException {

        Pose2d startPos = new Pose2d(12, 12, Math.PI/2);
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPos);


        TrajectoryActionBuilder auto = drive.actionBuilder(startPos)
                .lineToY(36)
                .waitSeconds(0.5)
                .lineToYLinearHeading(12,-Math.PI/2)
                .waitSeconds(0.5)
                .setTangent(Math.PI/2)
                .splineToLinearHeading(new Pose2d(17,31,-Math.PI*3/4),Math.PI/4)
                .splineToLinearHeading(new Pose2d(36,36,-Math.PI),0);



        waitForStart();
        if (isStopRequested()) return;
        Actions.runBlocking(auto.build());


    }


}

