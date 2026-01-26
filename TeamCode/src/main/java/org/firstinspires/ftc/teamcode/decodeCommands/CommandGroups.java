package org.firstinspires.ftc.teamcode.decodeCommands;

import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.ParallelRaceGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.decodeSubsystems.AutoShooter;
import org.firstinspires.ftc.teamcode.decodeSubsystems.CarouselSubsystem;
import org.firstinspires.ftc.teamcode.decodeSubsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.decodeSubsystems.Motif;

import java.util.function.Supplier;

public class CommandGroups {
    public static class Shoot extends ParallelRaceGroup {

        public Shoot(IntakeSubsystem intakeSubsystem, CarouselSubsystem carouselSubsystem) {
            addCommands(new SequentialCommandGroup(
                    new InstantCommand(() -> DischargeCommands.AutomaticAiming.shooting = true),
                    new IntakeCommands.TransferState(intakeSubsystem),
                    new WaitCommand(100),
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
        public PrepareShooting(IntakeSubsystem intakeSubsystem, CarouselSubsystem carouselSubsystem, MecanumDrive mecanumDrive, AutoShooter.TeamColor teamColor) {

            addCommands(
                    new InstantCommand(() -> DischargeCommands.AutomaticAiming.shooting = true),
                    new IntakeCommands.TransferState(intakeSubsystem),
                    new WaitCommand(100),
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
    }public static class StartOuttake extends ParallelCommandGroup {
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
        double rStart, lStart;

        public PowerTakeOff(MecanumDrive mecanumDrive, Supplier<Double> power) {
            this.mecanumDrive = mecanumDrive;
            this.power = power;
            addRequirements(mecanumDrive);
        }

        @Override
        public void initialize() {
            rStart = mecanumDrive.rightFront.getCurrentPosition();
            lStart = mecanumDrive.leftFront.getCurrentPosition();
            mecanumDrive.PTOMode();
            mecanumDrive.leftFront.setPower(-0.1);
            mecanumDrive.rightFront.setPower(-0.1);
            mecanumDrive.rightBack.setPower(0.1);
            mecanumDrive.leftBack.setPower(0.1);
        }

        @Override
        public void execute() {
            double power = this.power.get() - 0.01;
            power += Math.signum(power) * 0.1;

            if (mecanumDrive.rightFront.getCurrentPosition() - rStart > 537.6 / 4 +  mecanumDrive.leftFront.getCurrentPosition() - lStart){
                mecanumDrive.rightFront.setPower(-0.1);
                mecanumDrive.rightBack.setPower(0.1);
            }else{
                mecanumDrive.rightFront.setPower(-power);
                mecanumDrive.rightBack.setPower(power);
            }
            if (mecanumDrive.leftFront.getCurrentPosition() - lStart > 537.6 / 4 +  mecanumDrive.rightFront.getCurrentPosition() - rStart){
                mecanumDrive.leftFront.setPower(-0.1);
                mecanumDrive.leftBack.setPower(0.1);
            }else{
                mecanumDrive.leftFront.setPower(-power);
                mecanumDrive.leftBack.setPower(power);

            }

        }
    }

    public static class Sort extends ParallelRaceGroup {
        public Sort(IntakeSubsystem intakeSubsystem, CarouselSubsystem carouselSubsystem, Motif motif) {
            Supplier<Double> steps = () ->{
                int placement = carouselSubsystem.getGreenPlacement();
                if (placement == -1){
                    return 0.0;
                }
                switch (motif) {
                    case PGP:
                        return -(carouselSubsystem.getGreenPlacement() + 2.0) % 3;
                    case GPP:
                        return -(carouselSubsystem.getGreenPlacement() + 1.0) % 3.0;
                    case PPG:
                        return -carouselSubsystem.getGreenPlacement() % 3.0;
                }
                return 0.0;
            };


            addCommands(
                    new IntakeCommands.SortingState(intakeSubsystem),
                    new SequentialCommandGroup(
                            new CarouselCommands.SlideDistance(carouselSubsystem, -0.625, -0.8),
                            new WaitCommand(100)
                    )
            );
        }
    }


}
