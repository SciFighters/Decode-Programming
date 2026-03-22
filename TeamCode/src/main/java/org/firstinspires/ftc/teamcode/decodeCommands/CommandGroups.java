package org.firstinspires.ftc.teamcode.decodeCommands;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.ParallelRaceGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
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
                    new WaitUntilCommand(() -> {
                        PoseVelocity2d movement = mecanumDrive.localizer.update();
                        Pose2d pos = mecanumDrive.localizer.getPose();
                        com.seattlesolvers.solverslib.geometry.Vector2d movementEffect = new com.seattlesolvers.solverslib.geometry.Vector2d(
                                movement.linearVel.x, movement.linearVel.y).rotateBy(pos.heading.toDouble() / Math.PI * 180).times(0.5);
                        return AutoShooter.canLaunch(new Pose2d(pos.position.x + movementEffect.getX(), pos.position.y + movementEffect.getY(), pos.heading.toDouble()));
                    }),
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
        Supplier<Double> power;
        double rStart, lStart, rollStart;
        public static double right = 0, left = 0;

        public PowerTakeOff(MecanumDrive mecanumDrive, Supplier<Double> power) {
            this.mecanumDrive = mecanumDrive;
            this.power = power;
            addRequirements(mecanumDrive);
        }

        @Override
        public void initialize() {
            mecanumDrive.PTOMode();
            rStart = mecanumDrive.rightBack.getCurrentPosition();
            lStart = mecanumDrive.leftBack.getCurrentPosition();
            rollStart = mecanumDrive.lazyImu.get().getRobotYawPitchRollAngles().getRoll(AngleUnit.DEGREES);
        }

        @Override
        public void execute() {
            double power = this.power.get() + 0.01;
            power += Math.signum(power) * 0.1;

            right = Math.abs(mecanumDrive.rightBack.getCurrentPosition() - rStart);
            left = Math.abs(mecanumDrive.leftBack.getCurrentPosition() - lStart);

            double delta = -(mecanumDrive.lazyImu.get().getRobotYawPitchRollAngles().getRoll(AngleUnit.DEGREES) - rollStart) * 0.1;

            double rPower = Range.clip(power - delta, 0.1, power);
            double lPower = Range.clip(power + delta, 0.1, power);

            mecanumDrive.rightFront.setPower(rPower);
            mecanumDrive.rightBack.setPower(-rPower);

            mecanumDrive.leftFront.setPower(lPower);
            mecanumDrive.leftBack.setPower(-lPower);
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
