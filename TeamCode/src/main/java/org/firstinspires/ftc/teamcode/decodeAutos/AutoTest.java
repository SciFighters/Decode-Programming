package org.firstinspires.ftc.teamcode.decodeAutos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
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

        Pose2d startPos =new Pose2d(-38,56,0);
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPos);
//        TelemetryPacket fsd = new TelemetryPacket(false);
//        fsd.fieldOverlay().drawImage("background/season-2025-decode/field-2025-juice-dark.png", 0.0,0.0,5.530496,3.434);


        TrajectoryActionBuilder normalAuto = drive.actionBuilder(startPos,true)
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