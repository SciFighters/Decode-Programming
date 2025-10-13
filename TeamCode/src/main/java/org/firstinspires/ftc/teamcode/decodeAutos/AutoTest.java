package org.firstinspires.ftc.teamcode.decodeAutos;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.Subsystem;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.actions.ActionCommand;
import org.firstinspires.ftc.teamcode.decodeOpModes.AllianceDetectionOpMode;
import org.firstinspires.ftc.teamcode.decodeSubsystems.AutoShooter;
import org.firstinspires.ftc.teamcode.decodeSubsystems.LimelightSubsystem;

import java.util.HashSet;
import java.util.Set;

@Autonomous
public class AutoTest extends AllianceDetectionOpMode {
    final Pose2d startPose = new Pose2d(0,24, -Math.PI);

    @Override
    public void initialize() {
     mecanumDrive = new MecanumDrive(hardwareMap,startPose);
     limelightSubsystem = new LimelightSubsystem(hardwareMap, AutoShooter.TeamColor.RED,mecanumDrive);
     limelightSubsystem.startLimelight();
    }

    @Override
    public void initialize_loop() {
        super.initialize_loop();
        multipleTelemetry.addData("color",teamColor);
        multipleTelemetry.update();
    }

    @Override
    public void runInit() {
        TrajectoryActionBuilder gotoMiddle = mecanumDrive.actionBuilder(mecanumDrive.localizer.getPose(), teamColor == AutoShooter.TeamColor.BLUE)
                .setTangent(0)
                .splineToLinearHeading(new Pose2d(38,-33,0),0);
        Set<Subsystem> requirements = new HashSet<Subsystem>();
        requirements.add(mecanumDrive);
        CommandScheduler.getInstance().schedule(new ActionCommand(gotoMiddle.build(),requirements));
    }

    @Override
    public void end() {
        limelightSubsystem.stopLimelight();
    }
}