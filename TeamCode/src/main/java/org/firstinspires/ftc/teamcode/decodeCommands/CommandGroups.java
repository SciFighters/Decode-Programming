package org.firstinspires.ftc.teamcode.decodeCommands;

import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.ParallelRaceGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.decodeSubsystems.AutoShooter;
import org.firstinspires.ftc.teamcode.decodeSubsystems.CarouselSubsystem;
import org.firstinspires.ftc.teamcode.decodeSubsystems.IntakeSubsystem;

import java.util.function.Supplier;

public class CommandGroups {
    public static class Shoot extends ParallelRaceGroup {

        public Shoot(IntakeSubsystem intakeSubsystem, CarouselSubsystem carouselSubsystem) {
            addCommands(new SequentialCommandGroup(
                    new InstantCommand(() -> DischargeCommands.AutomaticAiming.shooting = true),
                    new IntakeCommands.TransferState(intakeSubsystem),
//                    new WaitCommand(100),
                    new WaitUntilCommand(() -> DischargeCommands.AutomaticAiming.inRange),
                    new CarouselCommands.Discharge(carouselSubsystem, intakeSubsystem),
                    new InstantCommand(() -> DischargeCommands.AutomaticAiming.shooting = false)
            ));
        }

        @Override
        public void end(boolean interrupted) {
            super.end(interrupted);
            if (interrupted) {
                DischargeCommands.AutomaticAiming.shooting = false;
            }
        }
    }

    public static class PrepareShooting extends SequentialCommandGroup {
        public PrepareShooting(IntakeSubsystem intakeSubsystem, CarouselSubsystem carouselSubsystem, MecanumDrive mecanumDrive) {

            addCommands(
                    new InstantCommand(() -> DischargeCommands.AutomaticAiming.shooting = true),
                    new IntakeCommands.TransferState(intakeSubsystem),
//                    new WaitCommand(100),
//                    new WaitCommand(300),
                    new WaitUntilCommand(() -> AutoShooter.canLaunch(mecanumDrive.localizer.getPose())),
                    new WaitUntilCommand(() -> DischargeCommands.AutomaticAiming.inRange),
                    new CarouselCommands.Discharge(carouselSubsystem, intakeSubsystem),
                    new InstantCommand(() -> DischargeCommands.AutomaticAiming.shooting = false)

            );
        }

        @Override
        public void end(boolean interrupted) {
            super.end(interrupted);
            if (interrupted) {
                DischargeCommands.AutomaticAiming.shooting = false;
            }
        }
    }

    public static class StartIntake extends ParallelCommandGroup {
        public StartIntake(IntakeSubsystem intakeSubsystem, CarouselSubsystem carouselSubsystem) {
            addCommands(
                    new IntakeCommands.IntakeState(intakeSubsystem),
                    new CarouselCommands.MoveToAngle(carouselSubsystem, 180)
            );
        }
    }

    public static class StartOuttake extends ParallelCommandGroup {
        public StartOuttake(IntakeSubsystem intakeSubsystem, CarouselSubsystem carouselSubsystem) {
            addCommands(
                    new IntakeCommands.OutTakeState(intakeSubsystem),
                    new CarouselCommands.MoveToAngle(carouselSubsystem, 190)
            );
        }
    }


    public static class PowerTakeOff extends CommandBase {
        MecanumDrive mecanumDrive;
        public static double power = 0;
        public static double left = 0, right = 0;

        public PowerTakeOff(MecanumDrive mecanumDrive) {
            this.mecanumDrive = mecanumDrive;
            addRequirements(mecanumDrive);
        }

        @Override
        public void initialize() {
            mecanumDrive.PTOMode();
            power = 0;
            right = 0;
            left = 0;
        }

        @Override
        public void execute() {
            double power = PowerTakeOff.power + 0.01;
            power += Math.signum(power) * 0.1;

            mecanumDrive.rightFront.setPower(power + right);
            mecanumDrive.rightBack.setPower(-power - right);

            mecanumDrive.leftFront.setPower(power + left);
            mecanumDrive.leftBack.setPower(-power - left);

        }
    }

    public static class SortedShooting extends SequentialCommandGroup {
        public SortedShooting(IntakeSubsystem intakeSubsystem, CarouselSubsystem carouselSubsystem, MecanumDrive mecanumDrive) {
            addCommands(
                    new InstantCommand(() -> DischargeCommands.AutomaticAiming.shooting = true),
                    new IntakeCommands.SortingState(intakeSubsystem),
                    new CarouselCommands.SlideDistance(carouselSubsystem, -0.56, 0.6).withTimeout(1000),
                    new WaitCommand(200),
                    new WaitUntilCommand(() -> AutoShooter.canLaunch(mecanumDrive.localizer.getPose())),
                    new WaitUntilCommand(() -> DischargeCommands.AutomaticAiming.inRange),
                    new CarouselCommands.SmartDischarge(carouselSubsystem, intakeSubsystem),
                    new InstantCommand(() -> DischargeCommands.AutomaticAiming.shooting = false)

            );
        }
    }


}
