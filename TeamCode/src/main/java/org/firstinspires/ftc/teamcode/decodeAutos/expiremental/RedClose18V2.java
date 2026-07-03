package org.firstinspires.ftc.teamcode.decodeAutos.expiremental;


import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.InstantCommand;
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

@Autonomous(name = "18 red close V2", group = "red close")
public class RedClose18V2 extends ActionOpMode {
    DischargeSubsystem dischargeSubsystem;
    IntakeSubsystem intakeSubsystem;
    CarouselSubsystem carouselSubsystem;
    MecanumDrive mecanumDrive;
    LimelightSubsystem limelightSubsystem;
    double iteration = 1;
    double turretStartAngle = 50;
    ElapsedTime time;

    @Override
    public void initialize() {
        time = new ElapsedTime();
        SavedValues.currentCount = 0;
        SavedValues.teamColor = AutoShooter.TeamColor.RED;
        Set<Subsystem> requirements = new HashSet<>();
        intakeSubsystem = new IntakeSubsystem(hardwareMap);
        dischargeSubsystem = new DischargeSubsystem(hardwareMap);
        dischargeSubsystem.resetTurret();
        carouselSubsystem = new CarouselSubsystem(hardwareMap);
        carouselSubsystem.resetEncoders();
        mecanumDrive = new MecanumDrive(hardwareMap, new Pose2d(-41.2, 54.3, 0));
        mecanumDrive.driveMode();
        limelightSubsystem = new LimelightSubsystem(hardwareMap, SavedValues.teamColor, mecanumDrive);
        boolean reversed = SavedValues.teamColor == AutoShooter.TeamColor.BLUE;
        requirements.add(mecanumDrive);
        TrajectoryActionBuilder shootPreload = mecanumDrive.actionBuilder(new Pose2d(-41.2, 54.3, 0), reversed)
                .setTangent(-Math.PI / 4)
                .splineToConstantHeading(new Vector2d(-30, 42), -Math.PI / 4);
        TrajectoryActionBuilder spikeOne = mecanumDrive.actionBuilder(new Pose2d(-30, 42, 0), reversed)
                .setTangent(Math.PI / 6)
                .splineToConstantHeading(new Vector2d(-18, 47), 0)//might need to wait
                .splineToConstantHeading(new Vector2d(-30, 42), -Math.PI * 7 / 8);
        TrajectoryActionBuilder spikeTwo = mecanumDrive.actionBuilder(new Pose2d(-30, 42, 0), reversed)
                .setTangent(Math.PI / 9)
                .splineToConstantHeading(new Vector2d(6, 47), 0)
                .setTangent(-Math.PI)
                .splineTo(new Vector2d(-14, 23), -Math.PI * 26 / 36);
        TrajectoryActionBuilder gateCycle = mecanumDrive.actionBuilder(new Pose2d(-14, 23, 0), reversed)
                .setTangent(Math.PI * 10 / 36)
//                .splineTo(new Vector2d(5,40),Math.PI * 18 / 36)
//                .splineToSplineHeading(new Pose2d(5,40.1,Math.PI /2),Math.PI /2)
                .splineToLinearHeading(new Pose2d(15, 62, Math.PI * 10.5 / 16), Math.PI / 2);
        TrajectoryActionBuilder pressGate = mecanumDrive.actionBuilder(new Pose2d(16, 62.5, Math.PI * 10.5 / 16), reversed)
                .setTangent(Math.PI * 3.5 / 4)
                .lineToX(12);
        TrajectoryActionBuilder gateCycleP2 = mecanumDrive.actionBuilder(new Pose2d(14, 60, Math.PI * 11 / 16))
                .setTangent(-Math.PI * 5 / 16)
                .splineTo(new Vector2d(-13, 20), -Math.PI * 4 / 5);
        TrajectoryActionBuilder spikeThree = mecanumDrive.actionBuilder(new Pose2d(-13, 20, Math.PI / 5))
                .setTangent(Math.PI / 5)
                .splineTo(new Vector2d(30, 47), 0)
                .setTangent(-Math.PI)
                .splineTo(new Vector2d(-13, 20), -Math.PI * 3 / 4);
        TrajectoryActionBuilder loadingZone = mecanumDrive.actionBuilder(new Pose2d(-13, 20, Math.PI / 4))
                .setTangent(Math.PI / 4)
                .splineTo(new Vector2d(46, 58), 0)
                .splineToSplineHeading(new Pose2d(46.1, 58, 0), -Math.PI)//outtake a bit
                .splineToConstantHeading(new Vector2d(-8, 16), -Math.PI * 3 / 4);
        TrajectoryActionBuilder park = mecanumDrive.actionBuilder(new Pose2d(-8, 16, 0))
                .setTangent(Math.PI / 4)
                .splineToConstantHeading(new Vector2d(0, 30), Math.PI / 2);

        CommandScheduler.getInstance().schedule(
                new ParallelCommandGroup(
                        new DischargeCommands.AutomaticAiming(dischargeSubsystem, limelightSubsystem, mecanumDrive, carouselSubsystem, SavedValues.teamColor),
                        new SequentialCommandGroup(
                                intakeSubsystem.open(),
                                new ParallelCommandGroup(
                                        new ActionCommand(shootPreload.build(), requirements),
                                        new CommandGroups.PrepareShooting(intakeSubsystem, carouselSubsystem, mecanumDrive)
                                ),
                                intakeSubsystem.open(),
                                new ParallelCommandGroup(
                                        new ActionCommand(spikeOne.build(), requirements),
                                        new SequentialCommandGroup(
                                                new CommandGroups.StartIntake(intakeSubsystem, carouselSubsystem).withTimeout(1500),
                                                new CommandGroups.PrepareShooting(intakeSubsystem, carouselSubsystem, mecanumDrive)
                                        )
                                ),
                                intakeSubsystem.open(),
                                new ParallelCommandGroup(
                                        new ActionCommand(spikeTwo.build(), requirements),
                                        new SequentialCommandGroup(
                                                new CommandGroups.StartIntake(intakeSubsystem, carouselSubsystem).withTimeout(1500),
                                                new CommandGroups.PrepareShooting(intakeSubsystem, carouselSubsystem, mecanumDrive)
                                        )
                                ),
                                intakeSubsystem.open(),
                                new ParallelRaceGroup(
                                        new SequentialCommandGroup(
                                                new ActionCommand(gateCycle.build(), requirements),
                                                new ParallelDeadlineGroup(
                                                        new WaitCommand(800),
                                                        new ActionCommand(pressGate.build(), requirements)
                                                )
                                        ),

                                        new CommandGroups.StartIntake(intakeSubsystem, carouselSubsystem)
                                ),


                                new ParallelCommandGroup(
                                        new ActionCommand(gateCycleP2.build(), requirements),
                                        new SequentialCommandGroup(
                                                new WaitCommand(200),
                                                intakeSubsystem.outtake().withTimeout(550),
                                                new CommandGroups.PrepareShooting(intakeSubsystem, carouselSubsystem, mecanumDrive))
                                ),


                                intakeSubsystem.open(),
                                new ParallelCommandGroup(
                                        new ActionCommand(spikeThree.build(), requirements),
                                        new SequentialCommandGroup(
                                                new CommandGroups.StartIntake(intakeSubsystem, carouselSubsystem).withTimeout(2300),
                                                new CommandGroups.StartOuttake(intakeSubsystem, carouselSubsystem).withTimeout(400),
                                                new CommandGroups.PrepareShooting(intakeSubsystem, carouselSubsystem, mecanumDrive)
                                        )
                                ),
                                intakeSubsystem.open(),
                                new ParallelCommandGroup(
                                        new ActionCommand(loadingZone.build(), requirements),
                                        new SequentialCommandGroup(
                                                new CommandGroups.StartIntake(intakeSubsystem, carouselSubsystem).withTimeout(2700),
                                                new CommandGroups.StartOuttake(intakeSubsystem, carouselSubsystem).withTimeout(500),
                                                new CommandGroups.PrepareShooting(intakeSubsystem, carouselSubsystem, mecanumDrive)
                                        )
                                ),


                                new ParallelCommandGroup(
                                        new ActionCommand(park.build(), requirements),
                                        intakeSubsystem.closeState()
                                )


                        )
                )
        );
        time.reset();
    }

    @Override
    public void initialize_loop() {
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
        }

    }

    @Override
    public void run() {
        super.run();
        SavedValues.position = mecanumDrive.localizer.getPose();
    }
}