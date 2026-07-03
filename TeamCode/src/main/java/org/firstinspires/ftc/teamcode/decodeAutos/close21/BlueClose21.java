package org.firstinspires.ftc.teamcode.decodeAutos.close21;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.ParallelDeadlineGroup;
import com.seattlesolvers.solverslib.command.ParallelRaceGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.Subsystem;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.actions.ActionCommand;
import org.firstinspires.ftc.teamcode.actions.ActionOpMode;
import org.firstinspires.ftc.teamcode.decodeCommands.CommandGroups;
import org.firstinspires.ftc.teamcode.decodeCommands.DischargeCommands;
import org.firstinspires.ftc.teamcode.decodeSubsystems.AutoShooter;
import org.firstinspires.ftc.teamcode.decodeSubsystems.CarouselSubsystem;
import org.firstinspires.ftc.teamcode.decodeSubsystems.DischargeSubsystem;
import org.firstinspires.ftc.teamcode.decodeSubsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.decodeSubsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.decodeSubsystems.SavedValues;

import java.util.HashSet;
import java.util.Set;

@Autonomous(name = "21 blue close", group = "blue close")
public class BlueClose21 extends ActionOpMode {
    DischargeSubsystem dischargeSubsystem;
    IntakeSubsystem intake;
    CarouselSubsystem carousel;
    MecanumDrive mecanumDrive;
    LimelightSubsystem limelightSubsystem;

    double turretStartAngle = 295;
    ElapsedTime time;

    @Override
    public void initialize() {
        time = new ElapsedTime();
        SavedValues.currentCount = 0;
        SavedValues.teamColor = AutoShooter.TeamColor.BLUE;
        Set<Subsystem> requirements = new HashSet<>();
        intake = new IntakeSubsystem(hardwareMap);
        dischargeSubsystem = new DischargeSubsystem(hardwareMap);
        dischargeSubsystem.resetTurret();
        carousel = new CarouselSubsystem(hardwareMap);
        carousel.resetEncoders();
        mecanumDrive = new MecanumDrive(hardwareMap, new Pose2d(-41.2, 54.3, 0));
        mecanumDrive.driveMode();
        limelightSubsystem = new LimelightSubsystem(hardwareMap, SavedValues.teamColor, mecanumDrive);
        boolean reversed = SavedValues.teamColor == AutoShooter.TeamColor.BLUE;
        requirements.add(mecanumDrive);
        TrajectoryActionBuilder wheatleyAutoOne = mecanumDrive.actionBuilder(mecanumDrive.localizer.getPose(), reversed)
                .setTangent(-Math.PI / 4)
                .splineToConstantHeading(new Vector2d(-18, 18), -Math.PI / 4);

        TrajectoryActionBuilder wheatleyAutoTwo = mecanumDrive.actionBuilder(new Pose2d(-18, 18, 0), reversed)
                .setTangent(0)
                .splineToLinearHeading(new Pose2d(16, 36, Math.PI / 2), Math.PI / 2)
                .splineToSplineHeading(new Pose2d(16, 36.1, Math.PI / 2), Math.PI / 2)
//                .splineToConstantHeading(new Vector2d(14, 48), Math.PI *5/ 8)
                .splineToConstantHeading(new Vector2d(-16, 7), -Math.PI * 3 / 4,null, new ProfileAccelConstraint(-60,60));
        TrajectoryActionBuilder wheatleyAutoTwoP2 = mecanumDrive.actionBuilder((new Pose2d(14, 44, Math.PI / 2)), reversed)
                .setTangent(-Math.PI / 2)
                .splineToConstantHeading(new Vector2d(-8, 12), -Math.PI * 3 / 4);

        TrajectoryActionBuilder midOneP1 = mecanumDrive.actionBuilder(new Pose2d(-8, 12, Math.PI / 2), reversed)
                .setTangent(Math.PI / 5)
//                .splineToConstantHeading(new Vector2d(12, 30), Math.PI / 3)
                .splineToLinearHeading(new Pose2d(14, 63.5, Math.PI * 12.1 / 18), Math.PI * 12.1 / 18);//15.5, 60 //13.5
        TrajectoryActionBuilder midTwoP1 = mecanumDrive.actionBuilder(new Pose2d(-13, 20, Math.PI / 2), reversed)
                .setTangent(Math.PI / 5)
                .splineToConstantHeading(new Vector2d(4, 30), Math.PI / 3)
                .splineToSplineHeading(new Pose2d(13, 62.5, Math.PI * 10.5 / 16), Math.PI / 2);
        TrajectoryActionBuilder pressGate = mecanumDrive.actionBuilder(new Pose2d(16, 62.5, Math.PI * 10.5 / 16), reversed)
                .setTangent(Math.PI * 3.5 / 4)
                .lineToX(12);

        TrajectoryActionBuilder midOneP2 = mecanumDrive.actionBuilder(new Pose2d(12, 57, Math.PI * 12.1 / 16), reversed)
                .setTangent(-Math.PI / 2)
                .splineToLinearHeading(new Pose2d(-8, 10, Math.PI / 2), -Math.PI * 5 / 8, null, new ProfileAccelConstraint(-100, 100));
        TrajectoryActionBuilder midTwoP2 = mecanumDrive.actionBuilder(new Pose2d(12, 57, Math.PI * 12.1 / 16), reversed)
                .setTangent(-Math.PI / 2)
                .splineToLinearHeading(new Pose2d(-13, 15, Math.PI / 2), -Math.PI * 6 / 8,null,new ProfileAccelConstraint(-100,100));

        TrajectoryActionBuilder wheatleyAutoThree = mecanumDrive.actionBuilder(new Pose2d(-13, 20, Math.PI / 2), reversed)
                .setTangent(Math.PI / 2)
                .splineTo(new Vector2d(-13, 46), Math.PI / 2)
                .splineToConstantHeading(new Vector2d(-13, 20), -Math.PI / 2);
        TrajectoryActionBuilder wheatleyAutoFour = mecanumDrive.actionBuilder(new Pose2d(-13, 20, Math.PI / 2), reversed)
                .setTangent(0)
                .splineToConstantHeading(new Vector2d(36, 28), Math.PI / 4)
                .splineToConstantHeading(new Vector2d(42, 52), Math.PI / 2)
                .splineToConstantHeading(new Vector2d(42, 52.1), -Math.PI / 2)
                .splineToConstantHeading(new Vector2d(-18, 10), -Math.PI * 3 / 4,null,new ProfileAccelConstraint(-60,60));

        TrajectoryActionBuilder prepareGate = mecanumDrive.actionBuilder(new Pose2d(-6, 16, Math.PI / 2), reversed)
                .splineToConstantHeading(new Vector2d(0, 25), Math.PI / 2);

        CommandScheduler.getInstance().schedule(
                new ParallelCommandGroup(
//                        new LimelightCommands.KalmanFilter(limelightSubsystem, mecanumDrive, dischargeSubsystem::getTurretAngle),
                        new DischargeCommands.AutomaticAiming(dischargeSubsystem, limelightSubsystem, mecanumDrive, carousel, SavedValues.teamColor),
                        new SequentialCommandGroup(
                                new ParallelCommandGroup(
                                        new ActionCommand(wheatleyAutoOne.build(), requirements),
                                        new SequentialCommandGroup(
//                                                new InstantCommand(() -> DischargeSubsystem.shooting = true),
//                                                new WaitCommand(600),
//                                                new InstantCommand(() -> DischargeSubsystem.shooting = false),
                                                new CommandGroups.PrepareShooting(intake, carousel, mecanumDrive)
                                        )
                                ),
                                new ParallelCommandGroup(
                                        new ActionCommand(wheatleyAutoTwo.build(), requirements),
                                        new SequentialCommandGroup(
                                                CommandGroups.startIntake(intake, carousel).withTimeout(2300),
                                                new CommandGroups.PrepareShooting(intake, carousel, mecanumDrive)

                                        )
                                ),

                                new ParallelRaceGroup(
                                        new SequentialCommandGroup(
                                                new ActionCommand(midOneP1.build(), requirements),
                                                new ParallelDeadlineGroup(
                                                        new WaitCommand(500)
//                                                        new ActionCommand(pressGate.build(), requirements)
                                                )
                                        ),

                                        CommandGroups.startIntake(intake, carousel)
                                ),


                                new ParallelCommandGroup(
                                        new ActionCommand(midOneP2.build(), requirements),
                                        new SequentialCommandGroup(
                                                CommandGroups.startIntake(intake, carousel).withTimeout(400),
                                                CommandGroups.startOuttake(intake, carousel).withTimeout(600),
                                                new CommandGroups.PrepareShooting(intake, carousel, mecanumDrive))
                                ),

                                new ParallelRaceGroup(
                                        new SequentialCommandGroup(
                                                new ActionCommand(midOneP1.build(), requirements),
                                                new ParallelDeadlineGroup(
                                                        new WaitCommand(500)
//                                                        ,
//                                                        new ActionCommand(pressGate.build(), requirements)
                                                )
                                        ),

                                        CommandGroups.startIntake(intake, carousel)
                                ),


                                new ParallelCommandGroup(
                                        new ActionCommand(midTwoP2.build(), requirements),
                                        new SequentialCommandGroup(
                                                CommandGroups.startIntake(intake, carousel).withTimeout(400),
                                                CommandGroups.startOuttake(intake, carousel).withTimeout(500),
                                                new CommandGroups.PrepareShooting(intake, carousel, mecanumDrive))
                                ),

                                new ParallelCommandGroup(
                                        new SequentialCommandGroup(
                                                CommandGroups.startIntake(intake, carousel).withTimeout(1400),
                                                CommandGroups.startOuttake(intake, carousel).withTimeout(300),
                                                new CommandGroups.PrepareShooting(intake, carousel, mecanumDrive)
                                        ),
                                        new ActionCommand(wheatleyAutoThree.build(), requirements)),


                                new ParallelRaceGroup(
                                        new SequentialCommandGroup(
                                                new ActionCommand(midOneP1.build(), requirements),
                                                new ParallelDeadlineGroup(
                                                        new WaitCommand(500)
//                                                        ,
//                                                        new ActionCommand(pressGate.build(), requirements)
                                                )
                                        ),

                                        CommandGroups.startIntake(intake, carousel)
                                ),

                                new ParallelCommandGroup(
                                        new ActionCommand(midTwoP2.build(), requirements),
                                        new SequentialCommandGroup(
                                                new ParallelRaceGroup(
                                                        CommandGroups.startIntake(intake, carousel),
                                                        new WaitCommand(900)
                                                ),
                                                CommandGroups.startOuttake(intake, carousel).withTimeout(500),
                                                new CommandGroups.PrepareShooting(intake, carousel, mecanumDrive))
                                ),


                                new ParallelRaceGroup(
                                        new SequentialCommandGroup(
                                                new ActionCommand(midOneP1.build(), requirements),
                                                new ParallelDeadlineGroup(
                                                        new WaitCommand(500)
//                                                        ,
//                                                        new ActionCommand(pressGate.build(), requirements)
                                                )
                                        ),

                                        CommandGroups.startIntake(intake, carousel)
                                ),

                                new ParallelCommandGroup(
                                        new ActionCommand(midTwoP2.build(), requirements),
                                        new SequentialCommandGroup(
                                                new ParallelRaceGroup(
                                                        CommandGroups.startIntake(intake, carousel),
                                                        new WaitCommand(900)
                                                ),
                                                CommandGroups.startOuttake(intake, carousel).withTimeout(500),
                                                new CommandGroups.PrepareShooting(intake, carousel, mecanumDrive))
                                ),

                                new ParallelCommandGroup(
                                        new ActionCommand(prepareGate.build(), requirements),
                                        CommandGroups.startIntake(intake, carousel)
                                )


                        )
                )
        );
        time.reset();
    }

    @Override
    public void initialize_loop() {
        limelightSubsystem.setPipeline(0);
        limelightSubsystem.startLimelight();
        if (time.seconds() > 3) {
            int current = limelightSubsystem.getMotif();
            SavedValues.startMotif = (current != -1) ? current : SavedValues.startMotif;
            double power = -(turretStartAngle - dischargeSubsystem.getTurretAngle()) * 0.018;
            if (Math.abs(turretStartAngle - dischargeSubsystem.getTurretAngle()) < 5) {
                power = 0;
            }
            power = Range.clip(power, -0.3, 0.3);
            dischargeSubsystem.setTurretPower(power);
            multipleTelemetry.addData("used", SavedValues.startMotif);
            multipleTelemetry.addData("current", current);
            multipleTelemetry.addData("heading", mecanumDrive.localizer.getPose().heading.toDouble() * 180 / Math.PI);
            multipleTelemetry.update();
//            mecanumDrive.localizer.setPose(new Pose2d(-41.2, -54.3, 0));
        }

    }

    @Override
    public void run() {
        limelightSubsystem.setPipeline(2);
        super.run();
        SavedValues.position = mecanumDrive.localizer.getPose();
    }

    @Override
    public void end() {

        intake.stopSensorThread();
    }
}
