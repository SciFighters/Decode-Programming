package org.firstinspires.ftc.teamcode.decodeOpModes.testers;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.actions.ActionOpMode;
import org.firstinspires.ftc.teamcode.decodeCommands.LimelightCommands;
import org.firstinspires.ftc.teamcode.decodeSubsystems.AutoShooter;
import org.firstinspires.ftc.teamcode.decodeSubsystems.LimelightSubsystem;
@TeleOp(group = "tests")
public class LimelightSubsystemTester extends ActionOpMode {
    MecanumDrive mecanumDrive;
    LimelightSubsystem limelightSubsystem;
    Pose2d startPos = new Pose2d(61.875,-60.25,-Math.PI);
    GamepadEx gamepad;
    @Override
    public void initialize() {
        gamepad = new GamepadEx(gamepad1);
        mecanumDrive = new MecanumDrive(hardwareMap,startPos);
        limelightSubsystem = new LimelightSubsystem(hardwareMap, AutoShooter.TeamColor.RED,mecanumDrive);
        schedule(new LimelightCommands.KalmanFilter(limelightSubsystem,mecanumDrive,() -> 0.0));
    }

    @Override
    public void run() {
        super.run();
        mecanumDrive.localizer.update();
        com.seattlesolvers.solverslib.geometry.Vector2d vector = new com.seattlesolvers.solverslib.geometry.Vector2d(
                gamepad.getLeftX(), gamepad.getLeftY() ).rotateBy(Math.toDegrees(-mecanumDrive.localizer.getPose().heading.toDouble() + Math.PI/2));
        com.acmerobotics.roadrunner.Vector2d vector2d = new com.acmerobotics.roadrunner.Vector2d(vector.getX(), vector.getY());
        mecanumDrive.setDrivePowers(new PoseVelocity2d(vector2d,-gamepad.getRightX()));
//        mecanumDrive.setDrivePowers(new PoseVelocity2d(new com.acmerobotics.roadrunner.Vector2d(-gamepad1.left_stick_y, -gamepad1.left_stick_x),-gamepad1.right_stick_x));
        Vector2d llPos = limelightSubsystem.getRobotPos(0);
        multipleTelemetry.addData("pinpointX",mecanumDrive.localizer.getPose().position.x);
        multipleTelemetry.addData("pinpointY",mecanumDrive.localizer.getPose().position.y);
        if(llPos != null){
            multipleTelemetry.addData("llPosX",llPos.getX());
            multipleTelemetry.addData("llPosY",llPos.getY());
        }
        multipleTelemetry.addData("gainX",LimelightCommands.KalmanFilter.kalmanGainX);
        multipleTelemetry.addData("gainY",LimelightCommands.KalmanFilter.kalmanGainY);
        multipleTelemetry.addData("kalmanX",LimelightCommands.KalmanFilter.position.getX());
        multipleTelemetry.addData("kalmanY",LimelightCommands.KalmanFilter.position.getY());
        multipleTelemetry.update();
    }

}
