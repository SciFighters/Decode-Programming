package org.firstinspires.ftc.teamcode.decodeAutos.expiremental;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
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
import com.seattlesolvers.solverslib.command.WaitUntilCommand;

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

@Autonomous()
public class RobotRevealAuto extends ActionOpMode {
    DischargeSubsystem dischargeSubsystem;
    IntakeSubsystem intakeSubsystem;
    CarouselSubsystem carouselSubsystem;
    MecanumDrive mecanumDrive;
    LimelightSubsystem limelightSubsystem;
    double iteration = 1;
    double turretStartAngle = 90;
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
        mecanumDrive = new MecanumDrive(hardwareMap, new Pose2d(61.5, 22, -Math.PI / 2));
        mecanumDrive.driveMode();
        limelightSubsystem = new LimelightSubsystem(hardwareMap, SavedValues.teamColor, mecanumDrive);
        boolean reversed = SavedValues.teamColor == AutoShooter.TeamColor.BLUE;
        requirements.add(mecanumDrive);
        TrajectoryActionBuilder wheatleyAutoOne = mecanumDrive.actionBuilder(mecanumDrive.localizer.getPose())
                .splineToConstantHeading(new Vector2d(61.4, -58), -Math.PI / 2)
                .splineToConstantHeading(new Vector2d(60, -30), Math.PI / 2);
        TrajectoryActionBuilder wheatleyAutoTwo = mecanumDrive.actionBuilder(new Pose2d(60, -30, -Math.PI / 2))
                .splineToConstantHeading(new Vector2d(61.4, -58), -Math.PI / 2)
                .splineToConstantHeading(new Vector2d(60, -30), Math.PI / 2);


        CommandScheduler.getInstance().schedule(
                new ParallelCommandGroup(
//                        new LimelightCommands.KalmanFilter(limelightSubsystem, mecanumDrive, dischargeSubsystem::getTurretAngle),
                        new DischargeCommands.AutomaticAiming(dischargeSubsystem, limelightSubsystem, mecanumDrive, carouselSubsystem, SavedValues.teamColor),
                        new SequentialCommandGroup(
                                new CommandGroups.Shoot(intakeSubsystem, carouselSubsystem),
                                new WaitUntilCommand(() -> dischargeSubsystem.getRPM() > 4000),
                                new WaitCommand(1),
                                new ParallelCommandGroup(
                                        new ActionCommand(wheatleyAutoOne.build(), requirements),
                                        new SequentialCommandGroup(
                                                new CommandGroups.StartIntake(intakeSubsystem, carouselSubsystem).withTimeout(3350),
                                                new CommandGroups.PrepareShooting(intakeSubsystem, carouselSubsystem, mecanumDrive)
                                        )

                                ), new ParallelCommandGroup(
                                new ActionCommand(wheatleyAutoTwo.build(), requirements),
                                new SequentialCommandGroup(
                                        new CommandGroups.StartIntake(intakeSubsystem, carouselSubsystem).withTimeout(1950),
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
        mecanumDrive.updatePoseEstimate();
        super.run();
        multipleTelemetry.addData("x", mecanumDrive.localizer.getPose().position.x);
        multipleTelemetry.addData("y", mecanumDrive.localizer.getPose().position.y);
        multipleTelemetry.update();
        SavedValues.position = mecanumDrive.localizer.getPose();
        SavedValues.turretAngle = dischargeSubsystem.getTurretAngle();
    }
}
