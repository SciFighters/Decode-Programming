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
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.Subsystem;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.actions.ActionCommand;

import java.util.HashSet;
import java.util.Set;


@Autonomous
public class AutoTest extends LinearOpMode {
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();


    @Override
    public void runOpMode() throws InterruptedException {

        Pose2d startPos =new Pose2d(-38,56,0);
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPos);



        TrajectoryActionBuilder normalAuto = drive.actionBuilder(new Pose2d(-38,56,0))
                .setTangent(-Math.PI / 4)
                .splineToLinearHeading(new Pose2d(-24,36,Math.PI / 4),-Math.PI/4)
                .waitSeconds(1)
                .setTangent(Math.PI / 4)
                .splineToSplineHeading(new Pose2d(-12,48,Math.PI/2),Math.PI/2)
                .splineToConstantHeading(new Vector2d(-6,55),Math.PI/4)
                .setTangent(Math.PI * 1.5)
                .splineTo(new Vector2d(-18,26),Math.PI* 10/9)
                .waitSeconds(1.5)
                .setTangent(Math.PI / 9)
                .splineTo(new Vector2d(12,54),Math.PI/2)
                .setTangent(-Math.PI/2)
                .splineTo(new Vector2d(-12,20),Math.PI)
                .waitSeconds(1.5)
                .setTangent(0)
                .splineTo(new Vector2d(36,54),Math.PI/2)
                .splineToSplineHeading(new Pose2d(36,56,Math.PI/2),Math.PI/2)
                .splineToConstantHeading(new Vector2d(56,16),-Math.PI/2)
                .waitSeconds(1);

        waitForStart();
        if (isStopRequested()) return;
        Actions.runBlocking(normalAuto.build());


    }


}

