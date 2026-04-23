package org.firstinspires.ftc.teamcode.decodeAutos.expiremental;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.MecanumDrive;
@Disabled
@Autonomous(name = "TestAuto(please disable)")
public class TestAuto extends LinearOpMode {
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();


    @Override
    public void runOpMode() throws InterruptedException {

        Pose2d startPos = new Pose2d(63, 0, -Math.PI);
        MecanumDrive mecanumDrive = new MecanumDrive(hardwareMap, startPos);


        TrajectoryActionBuilder auto = mecanumDrive.actionBuilder(startPos)
                .setTangent(-Math.PI)
                .splineToConstantHeading(new Vector2d(-20,0),-Math.PI,null,new ProfileAccelConstraint(-200,200));

        waitForStart();
        if (isStopRequested()) return;
        Actions.runBlocking(auto.build());


    }
}
