package org.firstinspires.ftc.teamcode.decodeOpModes.testers;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.actions.ActionOpMode;
import org.firstinspires.ftc.teamcode.decodeSubsystems.AutoShooter;
import org.firstinspires.ftc.teamcode.decodeSubsystems.LimelightSubsystem;

@TeleOp(group = "tests")
public class LimelightTester extends ActionOpMode {
    MecanumDrive mecanumDrive;
    LimelightSubsystem limelightSubsystem;
    @Override
    public void initialize() {
        limelightSubsystem = new LimelightSubsystem(hardwareMap, AutoShooter.TeamColor.RED,mecanumDrive);
//        limelightSubsystem.setPipeline(0);
        limelightSubsystem.startLimelight();
    }

    @Override
    public void run() {
        multipleTelemetry.addData("color",limelightSubsystem.getColorFromObelisk().toString());
        multipleTelemetry.update();
        super.run();
    }

    @Override
    public void end() {
        limelightSubsystem.stopLimelight();
    }
}
