package org.firstinspires.ftc.teamcode.decodeAutos.Far;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.Subsystem;

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

@Autonomous(group = "red far")
public class BlueFarCyclesFull extends ActionOpMode {
    DischargeSubsystem dischargeSubsystem;
    IntakeSubsystem intakeSubsystem;
    CarouselSubsystem carouselSubsystem;
    MecanumDrive mecanumDrive;
    LimelightSubsystem limelightSubsystem;
    double turretStartAngle = 90;
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
        mecanumDrive = new MecanumDrive(hardwareMap, new Pose2d(61.5, 22, Math.PI / 2));
        mecanumDrive.driveMode();
        limelightSubsystem = new LimelightSubsystem(hardwareMap, SavedValues.teamColor, mecanumDrive);
        boolean reversed = SavedValues.teamColor == AutoShooter.TeamColor.BLUE;
        SavedValues.zone = AutoShooter.Zone.FAR;
        requirements.add(mecanumDrive);
        // First intake off the wall: collect from the human player.
        TrajectoryActionBuilder humanPlayerIntake = mecanumDrive.actionBuilder(mecanumDrive.localizer.getPose(), reversed)
                .splineToConstantHeading(new Vector2d(61.4, 58), Math.PI / 2)
                .splineToConstantHeading(new Vector2d(60, 24), -Math.PI / 2);
        // Second intake: collect from the spike mark.
        TrajectoryActionBuilder spikeMarkIntake = mecanumDrive.actionBuilder(new Pose2d(58, 24, Math.PI / 2), reversed)
                .setTangent(Math.PI)
                .splineToConstantHeading(new Vector2d(30, 50), Math.PI / 2)
                .splineToConstantHeading(new Vector2d(58, 24), -Math.PI / 2);
        // Repeatable cycle for the rest of the auto: drive up to the human-player wall just
        // like the initial intake, back off, pivot in place to face along the wall, then drive
        // straight ~1.2 m while intake is running, and return to the shooting position to fire.
        // Each heading = direction of travel, such that the intake is always active;
        TrajectoryActionBuilder sweepIntake = mecanumDrive
                .actionBuilder(new Pose2d(58, 24, Math.PI / 2), reversed)
                .splineToConstantHeading(new Vector2d(61.4, 58), Math.PI / 2)   // up to the human-player wall, facing +Y
                .splineToConstantHeading(new Vector2d(58, 58), -Math.PI / 2)    // back off so it has room to turn
                .turnTo(Math.PI)                                                // pivot left in place to face along the wall
                .lineToX(11)                                                    // drive forward ~1.2 m (47 in), intake facing -X
                .setTangent(Math.toRadians(-30))
                .splineToLinearHeading(new Pose2d(58, 24, Math.PI / 2), Math.toRadians(-30)); // return to the shooting spot


        CommandScheduler.getInstance().schedule(
                new ParallelCommandGroup(
                        new DischargeCommands.AutomaticAiming(dischargeSubsystem, limelightSubsystem, mecanumDrive, carouselSubsystem, SavedValues.teamColor),
                        new SequentialCommandGroup(
                                new CommandGroups.Shoot(intakeSubsystem, carouselSubsystem),
                                // First intake off the wall: human player, then shoot.
                                new ParallelCommandGroup(
                                        new ActionCommand(humanPlayerIntake.build(), requirements),
                                        new SequentialCommandGroup(
                                                new CommandGroups.StartIntake(intakeSubsystem, carouselSubsystem).withTimeout(2500),
                                                new CommandGroups.PrepareShooting(intakeSubsystem, carouselSubsystem, mecanumDrive)
                                        )

                                ),
                                // Second intake: spike mark, then shoot.
                                new ParallelCommandGroup(
                                        new ActionCommand(spikeMarkIntake.build(), requirements),
                                        new SequentialCommandGroup(
                                                new CommandGroups.StartIntake(intakeSubsystem, carouselSubsystem).withTimeout(2000),
                                                new CommandGroups.PrepareShooting(intakeSubsystem, carouselSubsystem, mecanumDrive)
                                        )
                                ),
                                // Rest of the auto: repeat the wall-sweep intake -> shoot until time runs out.
                                new ParallelCommandGroup(
                                        new ActionCommand(sweepIntake.build(), requirements),
                                        new SequentialCommandGroup(
                                                new CommandGroups.StartIntake(intakeSubsystem, carouselSubsystem).withTimeout(4000),
                                                new CommandGroups.PrepareShooting(intakeSubsystem, carouselSubsystem, mecanumDrive)
                                        )
                                ),
                                new ParallelCommandGroup(
                                        new ActionCommand(sweepIntake.build(), requirements),
                                        new SequentialCommandGroup(
                                                new CommandGroups.StartIntake(intakeSubsystem, carouselSubsystem).withTimeout(4000),
                                                new CommandGroups.PrepareShooting(intakeSubsystem, carouselSubsystem, mecanumDrive)
                                        )
                                ),
                                new ParallelCommandGroup(
                                        new ActionCommand(sweepIntake.build(), requirements),
                                        new SequentialCommandGroup(
                                                new CommandGroups.StartIntake(intakeSubsystem, carouselSubsystem).withTimeout(4000),
                                                new CommandGroups.PrepareShooting(intakeSubsystem, carouselSubsystem, mecanumDrive)
                                        )
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
        limelightSubsystem.setPipeline(1);
        mecanumDrive.updatePoseEstimate();
        super.run();
        multipleTelemetry.addData("x", mecanumDrive.localizer.getPose().position.x);
        multipleTelemetry.addData("y", mecanumDrive.localizer.getPose().position.y);
        multipleTelemetry.update();
        SavedValues.position = mecanumDrive.localizer.getPose();
        SavedValues.turretAngle = dischargeSubsystem.getTurretAngle();
    }
}
