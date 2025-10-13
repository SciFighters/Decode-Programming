package org.firstinspires.ftc.teamcode.decodeOpModes.testers;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.actions.ActionOpMode;
import org.firstinspires.ftc.teamcode.decodeSubsystems.AutoShooter;
import org.firstinspires.ftc.teamcode.decodeSubsystems.LimelightSubsystem;
@TeleOp(group = "tests")
public class LimelightSubsystemTester extends ActionOpMode {
    MecanumDrive mecanumDrive;
    LimelightSubsystem limelightSubsystem;
    Pose2d startPos = new Pose2d(0,0,-Math.PI);
    @Override
    public void initialize() {
        mecanumDrive = new MecanumDrive(hardwareMap,startPos);
        limelightSubsystem = new LimelightSubsystem(hardwareMap, AutoShooter.TeamColor.RED,mecanumDrive);
        limelightSubsystem.setPipeline(1);
        limelightSubsystem.startLimelight();
    }

    @Override
    public void run() {
        mecanumDrive.localizer.update();
        mecanumDrive.setDrivePowers(new PoseVelocity2d(new com.acmerobotics.roadrunner.Vector2d(-gamepad1.left_stick_y, -gamepad1.left_stick_x),-gamepad1.right_stick_x));
        Vector2d llPos = limelightSubsystem.getRobotPos(mecanumDrive.localizer.getPose().heading.toDouble(),0);//todo: why not use internal mecanum drive
        multipleTelemetry.addData("pinpointx",mecanumDrive.localizer.getPose().position.x);
        multipleTelemetry.addData("pinpointy",mecanumDrive.localizer.getPose().position.y);
        multipleTelemetry.addData("llPosx",llPos.getX());
        multipleTelemetry.addData("llPosy",llPos.getY());
        multipleTelemetry.update();
        super.run();
    }

    @Override
    public void end() {
        limelightSubsystem.stopLimelight();
    }
}
