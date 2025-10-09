package org.firstinspires.ftc.teamcode.decodeAutos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.MecanumDrive;
@Autonomous(name = "15 far")
public class fifteenAuto extends LinearOpMode {
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();


    @Override
    public void runOpMode() throws InterruptedException {

        Pose2d startPos = new Pose2d(61, 23.0 + 1.0 / 3, Math.PI);
        MecanumDrive mecanumDrive = new MecanumDrive(hardwareMap, startPos);


        TrajectoryActionBuilder auto = mecanumDrive.actionBuilder(new Pose2d(61, 23.0 + 1.0 / 3, Math.PI))
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

        waitForStart();
        if (isStopRequested()) return;
        Actions.runBlocking(auto.build());


    }
}
