package org.firstinspires.ftc.teamcode.needle.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.ScheduleCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.Subsystem;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.actions.ActionCommand;
import org.firstinspires.ftc.teamcode.needle.commands.ArmAxisCommands;
import org.firstinspires.ftc.teamcode.needle.commands.Groups;
import org.firstinspires.ftc.teamcode.needle.commands.TelescopicArmCommands;
import org.firstinspires.ftc.teamcode.needle.subsystems.ArmAxisSubsystem;
import org.firstinspires.ftc.teamcode.needle.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.needle.subsystems.TelescopicArmSubsystem;

import java.util.HashSet;
import java.util.Set;

@Autonomous
public class KickoffAuto extends CommandOpMode {
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();
    public MecanumDrive drive;
    public TelescopicArmSubsystem telescopicArmSubsystem;
    public ArmAxisSubsystem armAxisSubsystem;
    public ClawSubsystem clawSubsystem;
    public Pose2d startPose = new Pose2d(12, -60, Math.PI / 2);
    public Set<Subsystem> fullSet, mecanumSet;

    @Override
    public void initialize() {
        drive = new MecanumDrive(hardwareMap, startPose);
        telescopicArmSubsystem = new TelescopicArmSubsystem(hardwareMap);
        armAxisSubsystem = new ArmAxisSubsystem(hardwareMap);
//        clawSubsystem = new ClawSubsystem(hardwareMap);
        fullSet = new HashSet<Subsystem>();
        fullSet.add(drive);
        fullSet.add(telescopicArmSubsystem);
        fullSet.add(armAxisSubsystem);
        mecanumSet = new HashSet<Subsystem>();
        mecanumSet.add(drive);
        TrajectoryActionBuilder first = drive.actionBuilder(startPose)
                .splineTo(new Vector2d(36, -12), 0) //stop for halfway spline
                .splineTo(new Vector2d(54, -50), -Math.PI / 2);//end spline
        TrajectoryActionBuilder second = drive.actionBuilder(new Pose2d(54, -50, -Math.PI / 2))
                .turn(-Math.PI);
        TrajectoryActionBuilder third = drive.actionBuilder(new Pose2d(54, -50, -Math.PI / 2))
                .setTangent(Math.PI/2)
                .splineTo(new Vector2d(36, -12), Math.PI) // halfway spline back
                .splineTo(new Vector2d(12, -60), -Math.PI / 2);
        TrajectoryActionBuilder fourth = drive.actionBuilder(startPose)
                .setTangent(Math.PI / 2)
                .splineTo(new Vector2d(56, 36), Math.PI / 2);
        TrajectoryActionBuilder fifth = drive.actionBuilder(new Pose2d(56, 36, Math.PI/2))
                .turn(Math.PI);
        TrajectoryActionBuilder last = drive.actionBuilder(new Pose2d(56, 36, 3 * Math.PI/2))
                .splineToLinearHeading(new Pose2d(12, -60,Math.toRadians(269.99)), -Math.PI / 2);
        CommandScheduler.getInstance().schedule(
                new ParallelCommandGroup(
                        new ArmAxisCommands.AxisControl(armAxisSubsystem, telescopicArmSubsystem, () -> 0.0),
                        new SequentialCommandGroup(
                                new ActionCommand(first.build(), fullSet),
                                new TelescopicArmCommands.GoTo(telescopicArmSubsystem, 60),
                                new WaitCommand(500),
                                new TelescopicArmCommands.GoTo(telescopicArmSubsystem, 30),
//                                new ActionCommand(second.build(), fullSet),
                                new ParallelCommandGroup(
                                        new Groups.GoToBasketCmd(telescopicArmSubsystem, armAxisSubsystem, clawSubsystem),
                                        new ActionCommand(third.build(), mecanumSet)
                                ),
                                new WaitCommand(1000),
                                new Groups.GoHome(telescopicArmSubsystem, armAxisSubsystem, clawSubsystem),
                                new ActionCommand(fourth.build(), mecanumSet),
                                new TelescopicArmCommands.GoTo(telescopicArmSubsystem, 60),
                                new WaitCommand(500),
                                new TelescopicArmCommands.GoTo(telescopicArmSubsystem, 30),
                                new ParallelCommandGroup(
                                        new ActionCommand(last.build(), mecanumSet),
                                        new ArmAxisCommands.AxisGoTo(armAxisSubsystem, telescopicArmSubsystem, 50),
                                        new TelescopicArmCommands.GoTo(telescopicArmSubsystem, 65)
                                )
                        )
                )
        );
    }

    @Override
    public void run() {
        super.run();
        drive.localizer.update();
        CommandScheduler.getInstance().run();
        telemetry.addData("x", drive.localizer.getPose().position.x);
        telemetry.addData("y", drive.localizer.getPose().position.y);
        telemetry.update();
    }

}
