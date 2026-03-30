package org.firstinspires.ftc.teamcode.decodeCommands;

import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SelectCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.decodeOpModes.testers.AutomaticShootingTuner;
import org.firstinspires.ftc.teamcode.decodeSubsystems.AutoShooter;
import org.firstinspires.ftc.teamcode.decodeSubsystems.CarouselSubsystem;
import org.firstinspires.ftc.teamcode.decodeSubsystems.DischargeSubsystem;
import org.firstinspires.ftc.teamcode.decodeSubsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.decodeSubsystems.SavedValues;

import java.util.HashMap;

public class CarouselCommands {


    public static class MoveToAngle extends CommandBase {
        private final CarouselSubsystem carouselSubsystem;
        double targetAngle;
        double currentAngle;
        double kp = 0.03;
        int tolerance = 3;
        private final boolean end;

        public MoveToAngle(CarouselSubsystem carouselSubsystem, double angle) {
            this.carouselSubsystem = carouselSubsystem;
            this.targetAngle = angle;
            end = false;
            addRequirements(carouselSubsystem);
        }

        public MoveToAngle(CarouselSubsystem carouselSubsystem, double angle, boolean end) {
            this.carouselSubsystem = carouselSubsystem;
            this.targetAngle = angle;
            this.end = end;
            addRequirements(carouselSubsystem);
        }


        @Override
        public void execute() {
            currentAngle = carouselSubsystem.getAngle();
            double error = targetAngle - currentAngle;
            double power = kp * error;
            carouselSubsystem.setSpinPower(power);
        }

        @Override
        public boolean isFinished() {
            return end && Math.abs(currentAngle - targetAngle) < tolerance;
        }


        @Override
        public void end(boolean interrupted) {
            carouselSubsystem.setSpinPower(0);
        }
    }


    public static class SlideDistance extends CommandBase {
        private final CarouselSubsystem carouselSubsystem;
        private double targetPos;
        int moveDirection;
        double power;
        double distance;

        public SlideDistance(CarouselSubsystem carouselSubsystem, double distance, double power) {//distance in thirds
            this.carouselSubsystem = carouselSubsystem;
            this.power = power;
            this.distance = distance;
            addRequirements(carouselSubsystem);
        }


        @Override
        public void initialize() {
            moveDirection = (int) Math.signum(distance);
            if (Math.signum(power) != moveDirection) {
                power *= -1;
            }

            targetPos = (carouselSubsystem.getPosition() + distance * carouselSubsystem.spinConversion);
        }

        @Override
        public void execute() {
            if (carouselSubsystem.getCurrent() > 7) {
                carouselSubsystem.setSpinPower(-power);
            } else {
                carouselSubsystem.setSpinPower(power);
            }
        }

        @Override
        public boolean isFinished() {
            return carouselSubsystem.getPosition() * moveDirection > targetPos * moveDirection;
        }

        @Override
        public void end(boolean interrupted) {
            carouselSubsystem.setSpinPower(0);
        }
    }


    public static class SmartDischarge extends SelectCommand {
        private static final double transferSpeed = 0.7, travelSpeed = 1;

        enum Sequence {
            ONE,//2,1,0
            TWO,//0,1,2
            THREE,//2,0,1
            FOUR,//0,2,1
            NONE
        }

        public SmartDischarge(CarouselSubsystem carouselSubsystem, IntakeSubsystem intakeSubsystem) {

            super(new HashMap<Object, Command>() {{
                put(Sequence.ONE, new SequentialCommandGroup(
                        new WaitUntilCommand(() -> DischargeCommands.AutomaticAiming.atCloseSpeed),
                        new IntakeCommands.TransferState(intakeSubsystem),
                        new SlideDistance(carouselSubsystem, -0.3, transferSpeed),
                        new WaitUntilCommand(() -> DischargeCommands.AutomaticAiming.atCloseSpeed),
                        new SlideDistance(carouselSubsystem, -0.6, transferSpeed),
                        new WaitUntilCommand(() -> DischargeCommands.AutomaticAiming.atSpeed),
                        new WaitCommand(200),
                        new SlideDistance(carouselSubsystem, -1, transferSpeed)
                ));
                put(Sequence.TWO, new SequentialCommandGroup(
                        new SlideDistance(carouselSubsystem, 1.6, travelSpeed),
                        new WaitUntilCommand(() -> DischargeCommands.AutomaticAiming.atCloseSpeed),
                        new IntakeCommands.TransferState(intakeSubsystem),
                        new SlideDistance(carouselSubsystem, 0.3, transferSpeed),
                        new WaitUntilCommand(() -> DischargeCommands.AutomaticAiming.atCloseSpeed),
                        new SlideDistance(carouselSubsystem, 0.6, transferSpeed),
                        new WaitUntilCommand(() -> DischargeCommands.AutomaticAiming.atSpeed),
                        new WaitCommand(200),
                        new SlideDistance(carouselSubsystem, 1, transferSpeed)
                ));
                put(Sequence.THREE, new SequentialCommandGroup(
                        new WaitUntilCommand(() -> DischargeCommands.AutomaticAiming.atCloseSpeed),
                        new IntakeCommands.TransferState(intakeSubsystem),
                        new SlideDistance(carouselSubsystem, -0.3, transferSpeed),
                        new SlideDistance(carouselSubsystem, 2, travelSpeed),
                        new WaitUntilCommand(() -> DischargeCommands.AutomaticAiming.atCloseSpeed),
                        new SlideDistance(carouselSubsystem, 0.5, transferSpeed),
//                        new SlideDistance(carouselSubsystem,-2,travelSpeed),
                        new WaitUntilCommand(() -> DischargeCommands.AutomaticAiming.atSpeed),
                        new WaitCommand(200),
                        new SlideDistance(carouselSubsystem, 1, transferSpeed)
                ));
                put(Sequence.FOUR, new SequentialCommandGroup(
                        new SlideDistance(carouselSubsystem, 1.6, travelSpeed),
                        new WaitUntilCommand(() -> DischargeCommands.AutomaticAiming.atCloseSpeed),
                        new IntakeCommands.TransferState(intakeSubsystem),
                        new SlideDistance(carouselSubsystem, 0.3, transferSpeed),
                        new SlideDistance(carouselSubsystem, -2.1, travelSpeed),
                        new WaitUntilCommand(() -> DischargeCommands.AutomaticAiming.atCloseSpeed),
                        new SlideDistance(carouselSubsystem, -0.6, transferSpeed),
                        new WaitUntilCommand(() -> DischargeCommands.AutomaticAiming.atSpeed),
                        new WaitCommand(200),
                        new SlideDistance(carouselSubsystem, -1, transferSpeed)
                ));
                put(Sequence.NONE, new SequentialCommandGroup(

                        new WaitUntilCommand(() -> DischargeCommands.AutomaticAiming.atCloseSpeed),
                        new IntakeCommands.TransferState(intakeSubsystem),
                        new SlideDistance(carouselSubsystem, -0.3, transferSpeed),
                        new WaitUntilCommand(() -> DischargeCommands.AutomaticAiming.atCloseSpeed),
                        new SlideDistance(carouselSubsystem, -0.6, transferSpeed),
                        new WaitUntilCommand(() -> DischargeCommands.AutomaticAiming.atCloseSpeed),
                        new SlideDistance(carouselSubsystem, -1, transferSpeed)
                ));
            }}, () -> getSequence(carouselSubsystem));
        }

        private static Sequence getSequence(CarouselSubsystem carouselSubsystem) {
            int wanted = (3 - SavedValues.currentCount % 3 + SavedValues.startMotif) % 3;
            int current = carouselSubsystem.getGreenPlacement();
            if ((current == 1 && wanted == 0) || current == -1) {
                return Sequence.NONE;
            }
            if (current + wanted == 2) {//1,1; 0,2; 2,0;
                return Sequence.ONE;
            } else if (current == wanted) {//0,0; 1,1;
                return Sequence.TWO;
            } else if (wanted == current + 1) {
                return Sequence.THREE;
            }
            return Sequence.FOUR;
        }


    }

    public static class Discharge extends SelectCommand {
        enum Sequence {
            CLOSE,
            MIDDLE,
            FAR

        }

        public Discharge(CarouselSubsystem carouselSubsystem) {
            super(new HashMap<Object, Command>() {{
                put(Sequence.CLOSE, new SequentialCommandGroup(
                        new InstantCommand(() -> DischargeSubsystem.shooting = false),
                        new WaitUntilCommand(() -> DischargeCommands.AutomaticAiming.atSpeed),
                        new SlideDistance(carouselSubsystem, 2.2, 1)
                ));
                put(Sequence.MIDDLE, new SequentialCommandGroup(

                        new SlideDistance(carouselSubsystem, 2, 0.95),
                        new WaitCommand(80),
                        new SlideDistance(carouselSubsystem, 1.2, 0.65)));
                put(Sequence.FAR, new SequentialCommandGroup(
                        new InstantCommand(() -> DischargeSubsystem.shooting = false),
                        new WaitUntilCommand(() -> DischargeCommands.AutomaticAiming.atCloseSpeed),
                        new InstantCommand(() -> DischargeSubsystem.shooting = true),
                        new SlideDistance(carouselSubsystem, 2.2, 1),
                        new InstantCommand(() -> DischargeSubsystem.shooting = false)));


            }}, () -> {
                double distance = AutoShooter.getGoalDistance(SavedValues.position, SavedValues.teamColor);
                if (distance < 115) {
                    return Sequence.CLOSE;
                }
                return Sequence.FAR;
            });
        }
    }


}
