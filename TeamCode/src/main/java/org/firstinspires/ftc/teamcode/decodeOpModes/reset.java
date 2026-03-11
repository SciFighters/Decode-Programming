package org.firstinspires.ftc.teamcode.decodeOpModes;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.decodeSubsystems.AutoShooter;
import org.firstinspires.ftc.teamcode.decodeSubsystems.CarouselSubsystem;
import org.firstinspires.ftc.teamcode.decodeSubsystems.DischargeSubsystem;
import org.firstinspires.ftc.teamcode.decodeSubsystems.SavedValues;

@Autonomous
public class reset extends LinearOpMode {
    DischargeSubsystem dischargeSubsystem;
    CarouselSubsystem carouselSubsystem;
    @Override
    public void runOpMode() throws InterruptedException {
        dischargeSubsystem = new DischargeSubsystem(hardwareMap);
        carouselSubsystem = new CarouselSubsystem(hardwareMap);
        waitForStart();
        dischargeSubsystem.resetTurret();
        carouselSubsystem.resetEncoders();
        SavedValues.position = new Pose2d(63,0,Math.PI);
        SavedValues.turretAngle = 180;
        SavedValues.teamColor = AutoShooter.TeamColor.BLUE;

    }
}
