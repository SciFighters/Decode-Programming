package org.firstinspires.ftc.teamcode.decodeAutos;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.ParallelRaceGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.Subsystem;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.actions.ActionCommand;
import org.firstinspires.ftc.teamcode.actions.ActionOpMode;
import org.firstinspires.ftc.teamcode.decodeCommands.CommandGroups;
import org.firstinspires.ftc.teamcode.decodeCommands.DischargeCommands;
import org.firstinspires.ftc.teamcode.decodeCommands.IntakeCommands;
import org.firstinspires.ftc.teamcode.decodeCommands.LimelightCommands;
import org.firstinspires.ftc.teamcode.decodeSubsystems.AutoShooter;
import org.firstinspires.ftc.teamcode.decodeSubsystems.CarouselSubsystem;
import org.firstinspires.ftc.teamcode.decodeSubsystems.DischargeSubsystem;
import org.firstinspires.ftc.teamcode.decodeSubsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.decodeSubsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.decodeSubsystems.SavedValues;

import java.util.HashSet;
import java.util.Set;

@Autonomous
public class NineAuto extends ActionOpMode {
    DischargeSubsystem dischargeSubsystem;
    IntakeSubsystem intakeSubsystem;
    CarouselSubsystem carouselSubsystem;
    MecanumDrive mecanumDrive;
    LimelightSubsystem limelightSubsystem;
    double iteration = 1;

    @Override
    public void initialize() {
        Set<Subsystem> requirements = new HashSet<>();
        intakeSubsystem = new IntakeSubsystem(hardwareMap);
        dischargeSubsystem = new DischargeSubsystem(hardwareMap);
        carouselSubsystem = new CarouselSubsystem(hardwareMap);
        carouselSubsystem.resetEncoders();
        mecanumDrive = new MecanumDrive(hardwareMap, new Pose2d(-47.2, 47.7, Math.PI));
        limelightSubsystem = new LimelightSubsystem(hardwareMap, SavedValues.teamColor, mecanumDrive);
        boolean reversed = SavedValues.teamColor == AutoShooter.TeamColor.BLUE;
        requirements.add(mecanumDrive);
        TrajectoryActionBuilder wheatleyAutoOne = mecanumDrive.actionBuilder(mecanumDrive.localizer.getPose(), reversed)
                .setTangent(-Math.PI / 4)
                .splineToLinearHeading(new Pose2d(-30, 30, Math.PI * 3 / 4), -Math.PI / 4);

        TrajectoryActionBuilder wheatleyAutoTwo = mecanumDrive.actionBuilder(new Pose2d(-30, 30, Math.PI * 3 / 4), reversed)
                .splineToLinearHeading(new Pose2d(-24, 24, Math.PI / 2), -Math.PI / 4)
                .splineToSplineHeading(new Pose2d(-23.9, 23.9, Math.PI / 2), Math.PI / 6)
//                .waitSeconds(1)
                .setTangent(Math.PI * 3 / 18)
                .splineToConstantHeading(new Vector2d(-11, 54), Math.PI / 2)
//                .splineToSplineHeading(new Pose2d(-11, 54.1, Math.PI / 2), -Math.PI / 2)
                .setTangent(-Math.PI / 2)
                .splineToLinearHeading(new Pose2d(-12, 14, Math.PI * 3 / 4), -Math.PI / 2);

        TrajectoryActionBuilder wheatleyAutoThree = mecanumDrive.actionBuilder(new Pose2d(-11, 14, Math.PI * 3 / 4), reversed)
                .setTangent(Math.PI / 6)
                .splineToSplineHeading(new Pose2d(14, 44, Math.PI / 2), Math.PI / 2)
                .splineToConstantHeading(new Vector2d(16, 54), Math.PI / 2)
                .splineToSplineHeading(new Pose2d(16, 54.1, Math.PI / 2), -Math.PI / 2)
//                .splineToSplineHeading(new Pose2d(14,50.1,Math.PI/2),-Math.PI / 2)
//                .splineToConstantHeading(new Vector2d(10, 54), Math.PI / 2)
//                .splineToSplineHeading(new Pose2d(10, 54.1, Math.PI / 2), -Math.PI / 2)
                .splineToLinearHeading(new Pose2d(-14, 14, Math.PI * 3 / 4), -Math.PI * 3 / 4);
        TrajectoryActionBuilder end = mecanumDrive.actionBuilder(new Pose2d(-14, 14, Math.PI * 3 / 4) )
                .setTangent(Math.PI /4)
                .splineToLinearHeading(new Pose2d(0,30,Math.PI / 2),Math.PI/2);
        CommandScheduler.getInstance().schedule(
                new ParallelCommandGroup(
                        new LimelightCommands.KalmanFilter(limelightSubsystem,mecanumDrive,dischargeSubsystem::getTurretAngle),
                        new DischargeCommands.AutomaticAiming(dischargeSubsystem, limelightSubsystem, mecanumDrive, carouselSubsystem, SavedValues.teamColor),
                        new SequentialCommandGroup(
                                new ActionCommand(wheatleyAutoOne.build(), requirements),
//                                new WaitCommand(4000),

                                new CommandGroups.Shoot(intakeSubsystem, carouselSubsystem),
//                                new WaitCommand(10000),
                                new ParallelRaceGroup(
                                        new ActionCommand(wheatleyAutoTwo.build(), requirements),
                                        new CommandGroups.StartIntake(intakeSubsystem, carouselSubsystem)
                                ),

                                new CommandGroups.Shoot(intakeSubsystem, carouselSubsystem),

                                new ParallelRaceGroup(
                                        new CommandGroups.StartIntake(intakeSubsystem, carouselSubsystem),
                                        new ActionCommand(wheatleyAutoThree.build(), requirements)
                                ),

                                new CommandGroups.Shoot(intakeSubsystem, carouselSubsystem),

                                new IntakeCommands.ClosedState(intakeSubsystem)


                        )
                )
        );

    }

    @Override
    public void run() {
        mecanumDrive.localizer.update();
        super.run();
        multipleTelemetry.addData("x", mecanumDrive.localizer.getPose().position.x);
        multipleTelemetry.addData("y", mecanumDrive.localizer.getPose().position.y);
        multipleTelemetry.update();
        SavedValues.position = mecanumDrive.localizer.getPose();
        SavedValues.carouselTicks = carouselSubsystem.getPosition();
    }
}
