package org.firstinspires.ftc.teamcode.decodeOpModes.testers;

import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.actions.ActionOpMode;
import org.firstinspires.ftc.teamcode.decodeSubsystems.AutoShooter;
import org.firstinspires.ftc.teamcode.decodeSubsystems.LimelightSubsystem;

import java.util.List;

@TeleOp(group = "tests")
public class LimelightTester extends ActionOpMode {
    MecanumDrive mecanumDrive;
    Limelight3A limelightSubsystem;
    @Override
    public void initialize() {
//        limelightSubsystem = new LimelightSubsystem(hardwareMap, AutoShooter.TeamColor.RED,mecanumDrive);
        limelightSubsystem = hardwareMap.get(Limelight3A.class,"limelight");
        limelightSubsystem.pipelineSwitch(0);
//        limelightSubsystem.setPipeline(0);
        limelightSubsystem.start();
    }

    @Override
    public void run() {
        List<LLResultTypes.FiducialResult> results = limelightSubsystem.getLatestResult().getFiducialResults();
        for (LLResultTypes.FiducialResult fiducialResult : results) {
            int id = fiducialResult.getFiducialId();
            if(21 <= id && id <= 23){
                multipleTelemetry.addData("x", fiducialResult.getCameraPoseTargetSpace().getPosition().x);
                multipleTelemetry.addData("y", fiducialResult.getCameraPoseTargetSpace().getPosition().y);
                multipleTelemetry.addData("z", fiducialResult.getCameraPoseTargetSpace().getPosition().z);
//                multipleTelemetry.addData("x", fiducialResult.getCameraPoseTargetSpace().getPosition().);
                multipleTelemetry.update();
            }
        }
//        multipleTelemetry.addData("color",);

        super.run();
    }

    @Override
    public void end() {
        limelightSubsystem.stop();
    }
}
