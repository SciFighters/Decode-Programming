package org.firstinspires.ftc.teamcode.decodeAutos;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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
@Autonomous(name = "15 red far")
public class FifteenAutoRed extends ActionOpMode {
    DischargeSubsystem dischargeSubsystem;
    IntakeSubsystem intakeSubsystem;
    CarouselSubsystem carouselSubsystem;
    MecanumDrive mecanumDrive;
    LimelightSubsystem limelightSubsystem;
    double iteration = 1;

    @Override
    public void initialize() {
        SavedValues.teamColor = AutoShooter.TeamColor.RED;
        Set<Subsystem> requirements = new HashSet<>();
        intakeSubsystem = new IntakeSubsystem(hardwareMap);
        dischargeSubsystem = new DischargeSubsystem(hardwareMap);
        dischargeSubsystem.resetTurret();
        carouselSubsystem = new CarouselSubsystem(hardwareMap);
        carouselSubsystem.resetEncoders();
        mecanumDrive = new MecanumDrive(hardwareMap, new Pose2d(61.5, 22, Math.PI / 2));
        limelightSubsystem = new LimelightSubsystem(hardwareMap, SavedValues.teamColor, mecanumDrive);
        boolean reversed = SavedValues.teamColor == AutoShooter.TeamColor.BLUE;
        requirements.add(mecanumDrive);
        TrajectoryActionBuilder wheatleyAutoOne = mecanumDrive.actionBuilder(mecanumDrive.localizer.getPose(), reversed)
                .splineToConstantHeading(new Vector2d(61.4, 58), Math.PI / 2)
                .splineToConstantHeading(new Vector2d(60.5, 24), -Math.PI / 2);

        TrajectoryActionBuilder wheatleyAutoTwo = mecanumDrive.actionBuilder(new Pose2d(60.5, 24, Math.PI / 2), reversed)
                .setTangent(Math.PI)
                .splineToSplineHeading(new Pose2d(20, 30, Math.PI * 11 / 18), Math.PI * 3 / 4)
                .splineToConstantHeading(new Vector2d(10, 46), Math.PI * 5 / 8)
                .splineToSplineHeading(new Pose2d(9.9,46.1,Math.PI * 11 / 18), Math.PI * 5 / 8)
                .splineToLinearHeading(new Pose2d(3, 52,Math.PI/2), Math.PI / 2);

        TrajectoryActionBuilder wheatleyAutoTwoP2 = mecanumDrive.actionBuilder((new Pose2d(4, 54, Math.PI / 2)), reversed)
                .setTangent(-Math.PI / 2)
                .splineToConstantHeading(new Vector2d(-12, 22), Math.PI);

        TrajectoryActionBuilder wheatleyAutoThree = mecanumDrive.actionBuilder(new Pose2d(-12, 22, Math.PI / 2), reversed)
                .setTangent(Math.PI / 2)
                .splineToConstantHeading(new Vector2d(-12, 46), Math.PI / 2)
                .splineToConstantHeading(new Vector2d(-12, 22), -Math.PI / 2);

        TrajectoryActionBuilder wheatleyAutoFour = mecanumDrive.actionBuilder(new Pose2d(-12, 22, Math.PI / 2), reversed)
                .setTangent(0)
                .splineToConstantHeading(new Vector2d(36, 48), Math.PI / 2)
                .splineToConstantHeading(new Vector2d(36,48.1),-Math.PI/2)
                .splineToConstantHeading(new Vector2d(-6, 16), -Math.PI * 3 / 4);

        TrajectoryActionBuilder prepareGate = mecanumDrive.actionBuilder(new Pose2d(-6, 16, Math.PI / 2), reversed)
                .setTangent(Math.PI * 3 / 8)
                .splineToConstantHeading(new Vector2d(0, 44), Math.PI / 2);

        CommandScheduler.getInstance().schedule(
                new ParallelCommandGroup(
//                        new LimelightCommands.KalmanFilter(limelightSubsystem, mecanumDrive, dischargeSubsystem::getTurretAngle),
                        new DischargeCommands.AutomaticAiming(dischargeSubsystem, limelightSubsystem, mecanumDrive, carouselSubsystem, SavedValues.teamColor),
                        new SequentialCommandGroup(
                                new CommandGroups.Shoot(intakeSubsystem,carouselSubsystem),
                                new ParallelCommandGroup(
                                        new ActionCommand(wheatleyAutoOne.build(), requirements),
                                        new SequentialCommandGroup(
                                                new CommandGroups.StartIntake(intakeSubsystem,carouselSubsystem).withTimeout(1850),
                                                new CommandGroups.PrepareShooting(intakeSubsystem, carouselSubsystem,mecanumDrive,SavedValues.teamColor)
                                        )

                                ),
                                new ParallelRaceGroup(
                                        new ActionCommand(wheatleyAutoTwo.build(), requirements),
                                        new CommandGroups.StartIntake(intakeSubsystem, carouselSubsystem)
                                ),
                                new WaitCommand(400),
                                new IntakeCommands.ClosedState(intakeSubsystem),
                                new WaitCommand(800),                                new ParallelCommandGroup(
                                        new ActionCommand(wheatleyAutoTwoP2.build(), requirements),
                                        new CommandGroups.PrepareShooting(intakeSubsystem, carouselSubsystem, mecanumDrive, SavedValues.teamColor)
                                ),

                                new ParallelCommandGroup(
                                        new SequentialCommandGroup(
                                                new CommandGroups.StartIntake(intakeSubsystem, carouselSubsystem).withTimeout(1300),
                                                new CommandGroups.PrepareShooting(intakeSubsystem, carouselSubsystem, mecanumDrive, SavedValues.teamColor)
                                        ),
                                        new ActionCommand(wheatleyAutoThree.build(), requirements)
                                ),
                                new ParallelCommandGroup(
                                        new SequentialCommandGroup(
                                                new CommandGroups.StartIntake(intakeSubsystem, carouselSubsystem).withTimeout(3500),
                                                new CommandGroups.PrepareShooting(intakeSubsystem, carouselSubsystem, mecanumDrive, SavedValues.teamColor)
                                        ),
                                        new ActionCommand(wheatleyAutoFour.build(), requirements)
                                ),
                                new ParallelCommandGroup(
                                        new ActionCommand(prepareGate.build(), requirements),
                                        new IntakeCommands.ClosedState(intakeSubsystem)
                                )


                        )
                )
        );

    }

    @Override
    public void run() {
        mecanumDrive.updatePoseEstimate();
        super.run();
        multipleTelemetry.addData("x", mecanumDrive.localizer.getPose().position.x);
        multipleTelemetry.addData("y", mecanumDrive.localizer.getPose().position.y);
        multipleTelemetry.update();
        SavedValues.position = mecanumDrive.localizer.getPose();
        SavedValues.turretAngle = dischargeSubsystem.getTurretAngle();
    }

    @Override
    public void end() {
        SavedValues.position = mecanumDrive.localizer.getPose();
    }
}
