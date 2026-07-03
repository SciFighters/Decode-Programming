package org.firstinspires.ftc.teamcode.decodeAutos.close12;

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
import org.firstinspires.ftc.teamcode.decodeSubsystems.AutoShooter;
import org.firstinspires.ftc.teamcode.decodeSubsystems.CarouselSubsystem;
import org.firstinspires.ftc.teamcode.decodeSubsystems.DischargeSubsystem;
import org.firstinspires.ftc.teamcode.decodeSubsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.decodeSubsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.decodeSubsystems.SavedValues;

import java.util.HashSet;
import java.util.Set;

@Autonomous(name = "12 blue close", group = "blue close")
public class BlueClose12 extends ActionOpMode {
    DischargeSubsystem dischargeSubsystem;
    IntakeSubsystem intake;
    CarouselSubsystem carousel;
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
                .splineToConstantHeading(new Vector2d(-22, 24), -Math.PI / 4);

        TrajectoryActionBuilder wheatleyAutoTwo = mecanumDrive.actionBuilder(new Pose2d(-22, 24, 0), reversed)
                .setTangent(0)
                .splineToLinearHeading(new Pose2d(14, 44, Math.PI / 2), Math.PI / 2)
                .splineToSplineHeading(new Pose2d(14, 44.1, Math.PI / 2), Math.PI / 2)
                .splineToConstantHeading(new Vector2d(5, 54), Math.PI / 2);
        TrajectoryActionBuilder wheatleyAutoTwoP2 = mecanumDrive.actionBuilder((new Pose2d(5, 54, Math.PI / 2)), reversed)
                .setTangent(-Math.PI / 2)
                .splineToConstantHeading(new Vector2d(-13, 20), -Math.PI);

        TrajectoryActionBuilder wheatleyAutoThree = mecanumDrive.actionBuilder(new Pose2d(-13, 20, Math.PI / 2), reversed)
                .setTangent(Math.PI / 2)
                .splineTo(new Vector2d(-13, 52), Math.PI / 2)
                .splineToConstantHeading(new Vector2d(-13, 20), -Math.PI / 2);

        TrajectoryActionBuilder wheatleyAutoFour = mecanumDrive.actionBuilder(new Pose2d(-13, 20, Math.PI / 2), reversed)
                .setTangent(0)
                .splineToConstantHeading(new Vector2d(30, 30), Math.PI / 18 * 6.5)

                .splineToConstantHeading(new Vector2d(36, 48), Math.PI / 2)
                .splineToConstantHeading(new Vector2d(-8, 14), -Math.PI * 3 / 4);
        TrajectoryActionBuilder prepareGate = mecanumDrive.actionBuilder(new Pose2d(-6, -16, -Math.PI / 2))
                .splineToConstantHeading(new Vector2d(0, -40), -Math.PI / 2);
        CommandScheduler.getInstance().schedule(
                new ParallelCommandGroup(
//                        new LimelightCommands.KalmanFilter(limelightSubsystem, mecanumDrive, dischargeSubsystem::getTurretAngle),
                        new DischargeCommands.AutomaticAiming(dischargeSubsystem, limelightSubsystem, mecanumDrive, carousel, SavedValues.teamColor),
                        new SequentialCommandGroup(
                                new ParallelCommandGroup(
                                        new ActionCommand(wheatleyAutoOne.build(), requirements),
                                        new SequentialCommandGroup(
                                                new WaitCommand(1500),
                                                new CommandGroups.PrepareShooting(intake, carousel, mecanumDrive)
                                        )
                                ),
                                new ParallelRaceGroup(
                                        new ActionCommand(wheatleyAutoTwo.build(), requirements),
                                        CommandGroups.startIntake(intake, carousel)
                                ),
                                new ParallelCommandGroup(
                                        new ActionCommand(wheatleyAutoTwoP2.build(), requirements),
                                        new CommandGroups.PrepareShooting(intake, carousel, mecanumDrive)
                                ),

                                new ParallelCommandGroup(
                                        new SequentialCommandGroup(
                                                CommandGroups.startIntake(intake, carousel).withTimeout(1400),
                                                new CommandGroups.PrepareShooting(intake, carousel, mecanumDrive)
                                        ),
                                        new ActionCommand(wheatleyAutoThree.build(), requirements)
                                ),
                                new ParallelCommandGroup(
                                        new SequentialCommandGroup(
                                                CommandGroups.startIntake(intake, carousel).withTimeout(3500),
                                                new CommandGroups.PrepareShooting(intake, carousel, mecanumDrive)
                                        ),
                                        new ActionCommand(wheatleyAutoFour.build(), requirements)
                                ),
                                new ParallelCommandGroup(
                                        new ActionCommand(prepareGate.build(), requirements),
                                        intake.closeState()
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
        mecanumDrive.updatePoseEstimate();
        super.run();
        multipleTelemetry.addData("x", mecanumDrive.localizer.getPose().position.x);
        multipleTelemetry.addData("y", mecanumDrive.localizer.getPose().position.y);
        multipleTelemetry.update();
        SavedValues.position = mecanumDrive.localizer.getPose();
        SavedValues.turretAngle = dischargeSubsystem.getTurretAngle();
    }
}
