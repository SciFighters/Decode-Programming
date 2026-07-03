package org.firstinspires.ftc.teamcode.decodeCommands;

import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SelectCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;
import com.seattlesolvers.solverslib.controller.PIDController;

import org.firstinspires.ftc.teamcode.decodeSubsystems.AutoShooter;
import org.firstinspires.ftc.teamcode.decodeSubsystems.CarouselSubsystem;
import org.firstinspires.ftc.teamcode.decodeSubsystems.DischargeSubsystem;
import org.firstinspires.ftc.teamcode.decodeSubsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.decodeSubsystems.SavedValues;

import java.util.HashMap;

public class CarouselCommands {
    public static final double CAROUSEL_STEP = 0.3;
    public static final double CAROUSEL_FULL = 1.0;
    public static final double CAROUSEL_HALF = 0.5;

    public static final long DEFAULT_TIMEOUT = 500; // millis


    public static class RotateDistance extends CommandBase {
        private final double SWITCH_DIRECTION_CURRENT = 7.0;

        private final CarouselSubsystem carousel;
        private double targetPos;
        int moveDirection;
        double power;
        double distance;

        public RotateDistance(CarouselSubsystem carousel, double distance, double power) { //distance in thirds
            this.carousel = carousel;
            this.power = power;
            this.distance = distance;
            addRequirements(carousel);
        }


        @Override
        public void initialize() {
            moveDirection = (int) Math.signum(distance);
            if (Math.signum(power) != Math.signum(distance)) {
                power *= -1;
            }

            targetPos = (carousel.getPosition() + distance * carousel.spinConversion);
        }

        @Override
        public void execute() {
            if (carousel.getCurrent() > SWITCH_DIRECTION_CURRENT) {
                carousel.setSpinPower(-power);
            } else {
                carousel.setSpinPower(power);
            }
        }

        @Override
        public boolean isFinished() {
            return carousel.getPosition() * moveDirection > targetPos * moveDirection;
        }

        @Override
        public void end(boolean interrupted) {
            carousel.setSpinPower(0);
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

        public SmartDischarge(CarouselSubsystem carousel, IntakeSubsystem intake) {

            super(new HashMap<Object, Command>() {{
                put(Sequence.ONE, new SequentialCommandGroup(
                        new WaitUntilCommand(() -> DischargeCommands.AutomaticAiming.atCloseSpeed).withTimeout(DEFAULT_TIMEOUT),
                        intake.transfer(),
                        new RotateDistance(carousel, -CAROUSEL_STEP, transferSpeed),
                        new WaitUntilCommand(() -> DischargeCommands.AutomaticAiming.atCloseSpeed),
                        new RotateDistance(carousel, -(CAROUSEL_STEP * 2), transferSpeed),
                        new WaitUntilCommand(() -> DischargeCommands.AutomaticAiming.atSpeed),
                        new WaitCommand(200),
                        new RotateDistance(carousel, -CAROUSEL_FULL, transferSpeed)
                ));
                put(Sequence.TWO, new SequentialCommandGroup(
                        new ParallelCommandGroup(
                                new RotateDistance(carousel, CAROUSEL_STEP * 2 + CAROUSEL_FULL, travelSpeed),
                                intake.transfer()
                        ),
                        new WaitUntilCommand(() -> DischargeCommands.AutomaticAiming.atCloseSpeed).withTimeout(DEFAULT_TIMEOUT),
                        new RotateDistance(carousel, CAROUSEL_STEP, transferSpeed),
                        new WaitUntilCommand(() -> DischargeCommands.AutomaticAiming.atCloseSpeed),
                        new RotateDistance(carousel, CAROUSEL_STEP * 2, transferSpeed),
                        new WaitUntilCommand(() -> DischargeCommands.AutomaticAiming.atSpeed),
                        new RotateDistance(carousel, CAROUSEL_FULL, transferSpeed)
                ));
                put(Sequence.THREE, new SequentialCommandGroup(
                        new WaitUntilCommand(() -> DischargeCommands.AutomaticAiming.atCloseSpeed).withTimeout(DEFAULT_TIMEOUT),
                        intake.transfer(),
                        new RotateDistance(carousel, -CAROUSEL_STEP, transferSpeed),
                        new RotateDistance(carousel, CAROUSEL_FULL * 2, travelSpeed),
                        new WaitUntilCommand(() -> DischargeCommands.AutomaticAiming.atCloseSpeed),
                        new RotateDistance(carousel, CAROUSEL_HALF, transferSpeed),
//                        new SlideDistance(carousel,-2,travelSpeed),
                        new WaitUntilCommand(() -> DischargeCommands.AutomaticAiming.atSpeed),
                        new WaitCommand(200),
                        new RotateDistance(carousel, CAROUSEL_FULL, transferSpeed)
                ));
                put(Sequence.FOUR, new SequentialCommandGroup(
                        new ParallelCommandGroup(
                                new RotateDistance(carousel, 1.2, travelSpeed),
                                intake.transfer()
                        ),
                        new WaitUntilCommand(() -> DischargeCommands.AutomaticAiming.atCloseSpeed).withTimeout(DEFAULT_TIMEOUT),
                        new RotateDistance(carousel, -1.8, travelSpeed),
                        new WaitUntilCommand(() -> DischargeCommands.AutomaticAiming.atCloseSpeed),
                        new RotateDistance(carousel, -CAROUSEL_STEP * 2, transferSpeed)
                ));
                put(Sequence.NONE, new SequentialCommandGroup(
                        new WaitUntilCommand(() -> DischargeCommands.AutomaticAiming.atCloseSpeed).withTimeout(DEFAULT_TIMEOUT),
                        intake.transfer(),
                        new RotateDistance(carousel, -CAROUSEL_STEP, transferSpeed),
                        new WaitUntilCommand(() -> DischargeCommands.AutomaticAiming.atCloseSpeed),
                        new RotateDistance(carousel, -CAROUSEL_STEP * 2, transferSpeed),
                        new WaitUntilCommand(() -> DischargeCommands.AutomaticAiming.atCloseSpeed),
                        new RotateDistance(carousel, -CAROUSEL_FULL, transferSpeed)
                ));
            }}, () -> getSequence(carousel));
        }

        private static Sequence getSequence(CarouselSubsystem carousel) {
            int wanted = (3 - SavedValues.currentCount % 3 + SavedValues.startMotif) % 3;
            int current = carousel.getGreenPlacement();
//            return Sequence.FOUR;
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

        public Discharge(CarouselSubsystem carousel) {
            super(new HashMap<Object, Command>() {{
                put(Sequence.CLOSE, new SequentialCommandGroup(
                        new InstantCommand(() -> DischargeSubsystem.shooting = false),
                        new WaitUntilCommand(() -> DischargeCommands.AutomaticAiming.atSpeed),
                        new RotateDistance(carousel, 2.2, 1)
                ));
                put(Sequence.MIDDLE, new SequentialCommandGroup(

                        new RotateDistance(carousel, 2, 0.95),
                        new WaitCommand(80),
                        new RotateDistance(carousel, 1.2, 0.65)));
                put(Sequence.FAR, new SequentialCommandGroup(
                        new InstantCommand(() -> DischargeSubsystem.shooting = false),
                        new WaitUntilCommand(() -> DischargeCommands.AutomaticAiming.atCloseSpeed),
                        new InstantCommand(() -> DischargeSubsystem.shooting = true),
                        new RotateDistance(carousel, 2.2, 1),
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