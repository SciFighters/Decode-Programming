package org.firstinspires.ftc.teamcode.decodeOpModes.testers;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.geometry.Vector2d;

import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.actions.ActionOpMode;
import org.firstinspires.ftc.teamcode.decodeCommands.LimelightCommands;
import org.firstinspires.ftc.teamcode.decodeSubsystems.AutoShooter;
import org.firstinspires.ftc.teamcode.decodeSubsystems.DischargeSubsystem;
import org.firstinspires.ftc.teamcode.decodeSubsystems.LimelightSubsystem;
@TeleOp(group = "tests")
public class LimelightSubsystemTester extends ActionOpMode {
    MecanumDrive mecanumDrive;
    LimelightSubsystem limelightSubsystem;
    DischargeSubsystem dischargeSubsystem;
    Pose2d startPos = new Pose2d(63,0,-Math.PI);
    @Override
    public void initialize() {
        mecanumDrive = new MecanumDrive(hardwareMap,startPos);
        limelightSubsystem = new LimelightSubsystem(hardwareMap, AutoShooter.TeamColor.RED,mecanumDrive);
        dischargeSubsystem = new DischargeSubsystem(hardwareMap);
        limelightSubsystem.setPipeline(1);
        limelightSubsystem.startLimelight();
        schedule(new LimelightCommands.KalmanFilter(limelightSubsystem,mecanumDrive,dischargeSubsystem::getTurretAngle));
    }

    @Override
    public void run() {
        mecanumDrive.localizer.update();
        mecanumDrive.setDrivePowers(new PoseVelocity2d(new com.acmerobotics.roadrunner.Vector2d(-gamepad1.left_stick_y, -gamepad1.left_stick_x),-gamepad1.right_stick_x));
        Position llPos = limelightSubsystem.getRobotPosMT2(dischargeSubsystem.getTurretAngle());
        if(llPos != null){
            multipleTelemetry.addData("llX",llPos.x);
            multipleTelemetry.addData("llY",llPos.y);
        }
        multipleTelemetry.addData("pinpointX",mecanumDrive.localizer.getPose().position.x);
        multipleTelemetry.addData("pinpointY",mecanumDrive.localizer.getPose().position.y);
        multipleTelemetry.addData("kalmanX", LimelightCommands.KalmanFilter.position.getX());
        multipleTelemetry.addData("kalmanY", LimelightCommands.KalmanFilter.position.getY());
        multipleTelemetry.update();
        super.run();
    }

    @Override
    public void end() {
        limelightSubsystem.stopLimelight();
    }
}
