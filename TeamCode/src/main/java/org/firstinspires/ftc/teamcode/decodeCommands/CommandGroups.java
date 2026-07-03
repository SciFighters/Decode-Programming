package org.firstinspires.ftc.teamcode.decodeCommands;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.ParallelRaceGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.decodeSubsystems.AutoShooter;
import org.firstinspires.ftc.teamcode.decodeSubsystems.CarouselSubsystem;
import org.firstinspires.ftc.teamcode.decodeSubsystems.IntakeSubsystem;

import java.util.function.Supplier;

public class CommandGroups {
    public static class Shoot extends SequentialCommandGroup {
        public Shoot(IntakeSubsystem intake, CarouselSubsystem carousel) {
            addCommands(
                    new InstantCommand(() -> DischargeCommands.AutomaticAiming.shooting = true),
                    intake.transfer(),
//                    new WaitCommand(100),
                    new WaitUntilCommand(() -> DischargeCommands.AutomaticAiming.inRange),
                    new CarouselCommands.Discharge(carousel),
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

    public static class PrepareShooting extends SequentialCommandGroup {
        public PrepareShooting(IntakeSubsystem intake, CarouselSubsystem carousel, MecanumDrive mecanumDrive) {

            addCommands(
                    new InstantCommand(() -> DischargeCommands.AutomaticAiming.shooting = true),
                    intake.transfer(),
                    new WaitUntilCommand(() -> {
                        PoseVelocity2d movement = mecanumDrive.localizer.update();
                        Pose2d pos = mecanumDrive.localizer.getPose();
                        com.seattlesolvers.solverslib.geometry.Vector2d movementEffect = new com.seattlesolvers.solverslib.geometry.Vector2d(
                                movement.linearVel.x, movement.linearVel.y).rotateBy(pos.heading.toDouble() / Math.PI * 180).times(0.35);
                        return AutoShooter.canLaunch(new Pose2d(pos.position.x + movementEffect.getX(), pos.position.y + movementEffect.getY(), pos.heading.toDouble()));
                    }),
                    new WaitUntilCommand(() -> DischargeCommands.AutomaticAiming.inRange),
                    new CarouselCommands.Discharge(carousel),
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

    public static Command startIntake(IntakeSubsystem intake, CarouselSubsystem carousel) {
        return new ParallelCommandGroup(
                intake.intake(),
                carousel.rotateToAngle(180)
        );
    }

    public static Command startOuttake(IntakeSubsystem intake, CarouselSubsystem carousel) {
        return new ParallelCommandGroup(
                intake.outtake(),
                carousel.rotateToAngle(195)
        );
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
            if (left > 5000 && right > 5000) {
                mecanumDrive.rightFront.setPower(0.1);
                mecanumDrive.rightBack.setPower(-0.1);
                mecanumDrive.leftFront.setPower(0.1);
                mecanumDrive.leftBack.setPower(-0.1);
            } else {
                mecanumDrive.rightFront.setPower(rPower);
                mecanumDrive.rightBack.setPower(-rPower);

                mecanumDrive.leftFront.setPower(lPower);
                mecanumDrive.leftBack.setPower(-lPower);
            }

        }
    }

    public static Command sortedShooting(IntakeSubsystem intake, CarouselSubsystem carousel) {
        return new SequentialCommandGroup(
                new InstantCommand(() -> DischargeCommands.AutomaticAiming.shooting = true),
                intake.sortState(),
                new CarouselCommands.RotateDistance(carousel, -0.56, 0.6).withTimeout(1000).whenFinished(() -> carousel.setSpinPower(0)),
//                    new WaitCommand(200),
                new WaitUntilCommand(() -> DischargeCommands.AutomaticAiming.inRange),
                new CarouselCommands.SmartDischarge(carousel, intake),
                new InstantCommand(() -> DischargeCommands.AutomaticAiming.shooting = false)
        );
    }
}