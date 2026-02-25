package org.firstinspires.ftc.teamcode.decodeAutos.expiremental;


import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
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
import org.firstinspires.ftc.teamcode.decodeSubsystems.AutoShooter;
import org.firstinspires.ftc.teamcode.decodeSubsystems.CarouselSubsystem;
import org.firstinspires.ftc.teamcode.decodeSubsystems.DischargeSubsystem;
import org.firstinspires.ftc.teamcode.decodeSubsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.decodeSubsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.decodeSubsystems.SavedValues;

import java.util.HashSet;
import java.util.Set;

@Autonomous(name = "18 blue close", group = "red close")
public class BlueClose extends ActionOpMode {
    DischargeSubsystem dischargeSubsystem;
    IntakeSubsystem intakeSubsystem;
    CarouselSubsystem carouselSubsystem;
    MecanumDrive mecanumDrive;
    LimelightSubsystem limelightSubsystem;
    double iteration = 1;
    double turretStartAngle = 310;
    ElapsedTime time;

    @Override
    public void initialize() {
        time = new ElapsedTime();
        SavedValues.currentCount = 0;
        SavedValues.teamColor = AutoShooter.TeamColor.BLUE;
        Set<Subsystem> requirements = new HashSet<>();
        intakeSubsystem = new IntakeSubsystem(hardwareMap);
        dischargeSubsystem = new DischargeSubsystem(hardwareMap);
        dischargeSubsystem.resetTurret();
        carouselSubsystem = new CarouselSubsystem(hardwareMap);
        carouselSubsystem.resetEncoders();
        mecanumDrive = new MecanumDrive(hardwareMap, new Pose2d(-41.2, 54.3, 0));
        limelightSubsystem = new LimelightSubsystem(hardwareMap, SavedValues.teamColor, mecanumDrive);
        boolean reversed = SavedValues.teamColor == AutoShooter.TeamColor.BLUE;
        requirements.add(mecanumDrive);
        TrajectoryActionBuilder wheatleyAutoOne = mecanumDrive.actionBuilder(mecanumDrive.localizer.getPose(), reversed)
                .setTangent(-Math.PI / 4)
                .splineToConstantHeading(new Vector2d(-18, 18), -Math.PI / 4);

        TrajectoryActionBuilder wheatleyAutoTwo = mecanumDrive.actionBuilder(new Pose2d(-18, 18, 0), reversed)
                .setTangent(0)
                .splineToLinearHeading(new Pose2d(14, 38, Math.PI / 2), Math.PI / 2)
                .splineToSplineHeading(new Pose2d(14, 38.1, Math.PI / 2), Math.PI / 2)
//                .splineToConstantHeading(new Vector2d(14, 48), Math.PI *5/ 8)
                .splineToConstantHeading(new Vector2d(-8, 12), -Math.PI * 3 / 4);
        TrajectoryActionBuilder wheatleyAutoTwoP2 = mecanumDrive.actionBuilder((new Pose2d(14, 44, Math.PI / 2)), reversed)
                .setTangent(-Math.PI / 2)
                .splineToConstantHeading(new Vector2d(-8, 12), -Math.PI * 3 / 4);

        TrajectoryActionBuilder midOneP1 = mecanumDrive.actionBuilder(new Pose2d(-8, 12, Math.PI / 2), reversed)
                .setTangent(Math.PI / 5)
                .splineToConstantHeading(new Vector2d(6, 30), Math.PI / 3)
                .splineToSplineHeading(new Pose2d(16, 62.5, Math.PI * 11 / 16), Math.PI / 2);//15.5, 60
        TrajectoryActionBuilder midTwoP1 = mecanumDrive.actionBuilder(new Pose2d(-13, 20, Math.PI / 2), reversed)
                .setTangent(Math.PI / 5)
                .splineToConstantHeading(new Vector2d(4, 30), Math.PI / 3)
                .splineToSplineHeading(new Pose2d(13, 62.5, Math.PI * 11 / 16), Math.PI / 2);
        TrajectoryActionBuilder pressGate = mecanumDrive.actionBuilder(new Pose2d(13,62.5,Math.PI * 11 / 16), reversed)
                .setTangent(Math.PI * 11 / 16)
                .lineToY(63.5);

        TrajectoryActionBuilder midOneP2 = mecanumDrive.actionBuilder(new Pose2d(14, 60, Math.PI * 5 / 8), reversed)
                .setTangent(-Math.PI / 2)
                .splineToLinearHeading(new Pose2d(-8, 12, Math.PI / 2), -Math.PI * 3 / 4);
        TrajectoryActionBuilder midTwoP2 = mecanumDrive.actionBuilder(new Pose2d(14, 60, Math.PI * 5 / 8), reversed)
                .setTangent(-Math.PI / 2)
                .splineToLinearHeading(new Pose2d(-13, 20, Math.PI / 2), -Math.PI * 3 / 4);

        TrajectoryActionBuilder wheatleyAutoThree = mecanumDrive.actionBuilder(new Pose2d(-13, 20, Math.PI / 2), reversed)
                .setTangent(Math.PI / 2)
                .splineTo(new Vector2d(-13, 46), Math.PI / 2)
                .splineToConstantHeading(new Vector2d(-13, 20), -Math.PI / 2);


        TrajectoryActionBuilder prepareGate = mecanumDrive.actionBuilder(new Pose2d(-6, -16, -Math.PI / 2))
                .splineToConstantHeading(new Vector2d(0, -40), -Math.PI / 2);
        CommandScheduler.getInstance().schedule(
                new ParallelCommandGroup(
//                        new LimelightCommands.KalmanFilter(limelightSubsystem, mecanumDrive, dischargeSubsystem::getTurretAngle),
                        new DischargeCommands.AutomaticAiming(dischargeSubsystem, limelightSubsystem, mecanumDrive, carouselSubsystem, SavedValues.teamColor),
                        new SequentialCommandGroup(
                                new ParallelCommandGroup(
                                        new ActionCommand(wheatleyAutoOne.build(), requirements),
                                        new SequentialCommandGroup(
//                                                new InstantCommand(() -> DischargeSubsystem.shooting = true),
//                                                new WaitCommand(600),
//                                                new InstantCommand(() -> DischargeSubsystem.shooting = false),
                                                new CommandGroups.PrepareShooting(intakeSubsystem, carouselSubsystem, mecanumDrive)
                                        )
                                ),
                                new ParallelCommandGroup(
                                        new ActionCommand(wheatleyAutoTwo.build(), requirements),
                                        new SequentialCommandGroup(
                                                new CommandGroups.StartIntake(intakeSubsystem, carouselSubsystem).withTimeout(2300),
                                                new CommandGroups.PrepareShooting(intakeSubsystem, carouselSubsystem, mecanumDrive)

                                        )
                                ),
//                                new ParallelCommandGroup(
//                                        new ActionCommand(wheatleyAutoTwoP2.build(), requirements),
//                                        new CommandGroups.PrepareShooting(intakeSubsystem, carouselSubsystem, mecanumDrive)
//                                ),
//                                new WaitCommand(500),
                                new ParallelRaceGroup(
                                        new ActionCommand(midOneP1.build(), requirements),
                                        new CommandGroups.StartIntake(intakeSubsystem, carouselSubsystem)
                                ),
                                new ParallelCommandGroup(
                                        new ActionCommand(pressGate.build(),requirements).withTimeout(450),
                                        new WaitCommand(450)
                                ),


                                new ParallelCommandGroup(
                                        new ActionCommand(midOneP2.build(), requirements),
                                        new SequentialCommandGroup(
                                                new WaitCommand(400),
                                                new CommandGroups.StartOuttake(intakeSubsystem, carouselSubsystem).withTimeout(500),
                                                new CommandGroups.PrepareShooting(intakeSubsystem, carouselSubsystem, mecanumDrive))
                                ),


                                new ParallelRaceGroup(
                                        new ActionCommand(midOneP1.build(), requirements),
                                        new CommandGroups.StartIntake(intakeSubsystem, carouselSubsystem)
                                ),
                                new ParallelCommandGroup(
                                        new ActionCommand(pressGate.build(),requirements).withTimeout(650),
                                        new WaitCommand(650)
                                ),

                                new ParallelCommandGroup(
                                        new ActionCommand(midOneP2.build(), requirements),
                                        new SequentialCommandGroup(
                                                new WaitCommand(400),
                                                new CommandGroups.StartOuttake(intakeSubsystem, carouselSubsystem).withTimeout(500),
                                                new CommandGroups.PrepareShooting(intakeSubsystem, carouselSubsystem, mecanumDrive))
                                ),
                                new ParallelRaceGroup(
                                        new ActionCommand(midOneP1.build(), requirements),
                                        new CommandGroups.StartIntake(intakeSubsystem, carouselSubsystem)
                                ),
                                new ParallelCommandGroup(
                                        new ActionCommand(pressGate.build(),requirements).withTimeout(650),
                                        new WaitCommand(650)
                                ),

                                new ParallelCommandGroup(
                                        new ActionCommand(midTwoP2.build(), requirements),
                                        new SequentialCommandGroup(
                                                new WaitCommand(400),
                                                new CommandGroups.StartOuttake(intakeSubsystem, carouselSubsystem).withTimeout(500),
                                                new CommandGroups.PrepareShooting(intakeSubsystem, carouselSubsystem, mecanumDrive))
                                ),
                                new ParallelCommandGroup(
                                        new SequentialCommandGroup(
                                                new CommandGroups.StartIntake(intakeSubsystem, carouselSubsystem).withTimeout(1400),
                                                new CommandGroups.PrepareShooting(intakeSubsystem, carouselSubsystem, mecanumDrive)
                                        ),
                                        new ActionCommand(wheatleyAutoThree.build(), requirements)),

                                new ParallelCommandGroup(
                                        new ActionCommand(prepareGate.build(), requirements),
                                        new IntakeCommands.ClosedState(intakeSubsystem)
                                )


                        )
                )
        );
        time.reset();
    }

    @Override
    public void initialize_loop() {
        limelightSubsystem.startLimelight();
        if (time.seconds() > 1) {
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
            multipleTelemetry.update();
        }

    }

    @Override
    public void run() {
        super.run();
        SavedValues.position = mecanumDrive.localizer.getPose();
    }
}

