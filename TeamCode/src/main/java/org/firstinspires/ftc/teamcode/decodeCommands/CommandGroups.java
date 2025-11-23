package org.firstinspires.ftc.teamcode.decodeCommands;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.ParallelRaceGroup;
import com.seattlesolvers.solverslib.command.PerpetualCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.decodeSubsystems.AutoShooter;
import org.firstinspires.ftc.teamcode.decodeSubsystems.CarouselSubsystem;
import org.firstinspires.ftc.teamcode.decodeSubsystems.DischargeSubsystem;
import org.firstinspires.ftc.teamcode.decodeSubsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.needle.commands.MecanumCommands;

import java.util.function.Supplier;

public class CommandGroups {
    public static class Shoot extends ParallelRaceGroup {
        public Shoot(IntakeSubsystem intakeSubsystem, CarouselSubsystem carouselSubsystem, MecanumDrive mecanumDrive, AutoShooter.TeamColor teamColor, Supplier<Double> x, Supplier<Double> y) {
            boolean far = AutoShooter.getGoalDistance(mecanumDrive.localizer.getPose(), teamColor) > 110;
            addCommands(
                    new MecanumCommands.Drive(mecanumDrive, x, y, () -> {
                        double launchAngle = ((AutoShooter.getLaunchAngle(new Pose2d(mecanumDrive.localizer.getPose().position, 0), teamColor)) % 360) / 180 * Math.PI;
                        double error = launchAngle - mecanumDrive.localizer.getPose().heading.toDouble();
                        return -(error * 0.65 + Math.signum(error) * 0.075);
                    },teamColor),
                    new SequentialCommandGroup(
                            new InstantCommand(() -> DischargeCommands.AutomaticAiming.shooting = true),
                            new IntakeCommands.TransferState(intakeSubsystem),
                            new WaitCommand(1000),
                            new CarouselCommands.SmartDischarge(carouselSubsystem, intakeSubsystem,far),
                            new InstantCommand(() -> DischargeCommands.AutomaticAiming.shooting = false)
                    )
            );
        }
        public Shoot(IntakeSubsystem intakeSubsystem,CarouselSubsystem carouselSubsystem){
            addCommands(new SequentialCommandGroup(
                    new InstantCommand(() -> DischargeCommands.AutomaticAiming.shooting = true),
                    new IntakeCommands.TransferState(intakeSubsystem),
                    new WaitCommand(1000),
                    new CarouselCommands.SmartDischarge(carouselSubsystem,intakeSubsystem,false),
                    new InstantCommand(() -> DischargeCommands.AutomaticAiming.shooting = false)
            ));
        }

        @Override
        public void end(boolean interrupted) {
            super.end(interrupted);
            if(interrupted){
                DischargeCommands.AutomaticAiming.shooting = false;
            }
        }
    }
    public static class StartIntake extends ParallelCommandGroup{
        public StartIntake(IntakeSubsystem intakeSubsystem, CarouselSubsystem carouselSubsystem){
            addCommands(
                    new IntakeCommands.IntakeState(intakeSubsystem),
                    new CarouselCommands.MoveToAngle(carouselSubsystem, 180)
            );
        }
    }



}
