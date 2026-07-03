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

    public static class MoveToAngle extends CommandBase {
        private final CarouselSubsystem carouselSubsystem;
        private final double targetAngle;
        private double currentAngle;
        private final PIDController pid = new PIDController(0.018, 0, 0);
        private int tolerance = 3;
        private final boolean finishWhenAtSetpoint;

        public MoveToAngle(CarouselSubsystem carouselSubsystem, double angle) {
            this.carouselSubsystem = carouselSubsystem;
            this.targetAngle = angle;
            finishWhenAtSetpoint = false;
            addRequirements(carouselSubsystem);
        }

        public MoveToAngle(CarouselSubsystem carouselSubsystem, double angle, boolean finishWhenAtSetpoint) {
            this.carouselSubsystem = carouselSubsystem;
            this.targetAngle = angle;
            this.finishWhenAtSetpoint = finishWhenAtSetpoint;
            addRequirements(carouselSubsystem);
        }


        @Override
        public void execute() {
            currentAngle = carouselSubsystem.getAngle();
            double power = pid.calculate(currentAngle, targetAngle);
            carouselSubsystem.setSpinPower(power);
        }

        @Override
        public boolean isFinished() {
            return finishWhenAtSetpoint && Math.abs(currentAngle - targetAngle) < tolerance;
        }


        @Override
        public void end(boolean interrupted) {
            carouselSubsystem.stop();
        }
    }

    public static class RotateDistance extends CommandBase {
        private final double SWITCH_DIRECTION_CURRENT = 7.0;

        private final CarouselSubsystem carouselSubsystem;
        private double targetPos;
        int moveDirection;
        double power;
        double distance;

        public RotateDistance(CarouselSubsystem carouselSubsystem, double distance, double power) { //distance in thirds
            this.carouselSubsystem = carouselSubsystem;
            this.power = power;
            this.distance = distance;
            addRequirements(carouselSubsystem);
        }


        @Override
        public void initialize() {
            moveDirection = (int) Math.signum(distance);
            if (Math.signum(power) != Math.signum(distance)) {
                power *= -1;
            }

            targetPos = (carouselSubsystem.getPosition() + distance * carouselSubsystem.spinConversion);
        }

        @Override
        public void execute() {
            if (carouselSubsystem.getCurrent() > SWITCH_DIRECTION_CURRENT) {
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
                        new WaitUntilCommand(() -> DischargeCommands.AutomaticAiming.atCloseSpeed).withTimeout(DEFAULT_TIMEOUT),
                        intakeSubsystem.transfer(),
                        new RotateDistance(carouselSubsystem, -CAROUSEL_STEP, transferSpeed),
                        new WaitUntilCommand(() -> DischargeCommands.AutomaticAiming.atCloseSpeed),
                        new RotateDistance(carouselSubsystem, -(CAROUSEL_STEP * 2), transferSpeed),
                        new WaitUntilCommand(() -> DischargeCommands.AutomaticAiming.atSpeed),
                        new WaitCommand(200),
                        new RotateDistance(carouselSubsystem, -CAROUSEL_FULL, transferSpeed)
                ));
                put(Sequence.TWO, new SequentialCommandGroup(
                        new ParallelCommandGroup(
                                new RotateDistance(carouselSubsystem, CAROUSEL_STEP * 2 + CAROUSEL_FULL, travelSpeed),
                                intakeSubsystem.transfer()
                        ),
                        new WaitUntilCommand(() -> DischargeCommands.AutomaticAiming.atCloseSpeed).withTimeout(DEFAULT_TIMEOUT),
                        new RotateDistance(carouselSubsystem, CAROUSEL_STEP, transferSpeed),
                        new WaitUntilCommand(() -> DischargeCommands.AutomaticAiming.atCloseSpeed),
                        new RotateDistance(carouselSubsystem, CAROUSEL_STEP * 2, transferSpeed),
                        new WaitUntilCommand(() -> DischargeCommands.AutomaticAiming.atSpeed),
                        new RotateDistance(carouselSubsystem, CAROUSEL_FULL, transferSpeed)
                ));
                put(Sequence.THREE, new SequentialCommandGroup(
                        new WaitUntilCommand(() -> DischargeCommands.AutomaticAiming.atCloseSpeed).withTimeout(DEFAULT_TIMEOUT),
                        intakeSubsystem.transfer(),
                        new RotateDistance(carouselSubsystem, -CAROUSEL_STEP, transferSpeed),
                        new RotateDistance(carouselSubsystem, CAROUSEL_FULL * 2, travelSpeed),
                        new WaitUntilCommand(() -> DischargeCommands.AutomaticAiming.atCloseSpeed),
                        new RotateDistance(carouselSubsystem, CAROUSEL_HALF, transferSpeed),
//                        new SlideDistance(carouselSubsystem,-2,travelSpeed),
                        new WaitUntilCommand(() -> DischargeCommands.AutomaticAiming.atSpeed),
                        new WaitCommand(200),
                        new RotateDistance(carouselSubsystem, CAROUSEL_FULL, transferSpeed)
                ));
                put(Sequence.FOUR, new SequentialCommandGroup(
                        new ParallelCommandGroup(
                                new RotateDistance(carouselSubsystem, 1.2, travelSpeed),
                                intakeSubsystem.transfer()
                        ),
                        new WaitUntilCommand(() -> DischargeCommands.AutomaticAiming.atCloseSpeed).withTimeout(DEFAULT_TIMEOUT),
                        new RotateDistance(carouselSubsystem, -1.8, travelSpeed),
                        new WaitUntilCommand(() -> DischargeCommands.AutomaticAiming.atCloseSpeed),
                        new RotateDistance(carouselSubsystem, -CAROUSEL_STEP * 2, transferSpeed)
                ));
                put(Sequence.NONE, new SequentialCommandGroup(
                        new WaitUntilCommand(() -> DischargeCommands.AutomaticAiming.atCloseSpeed).withTimeout(DEFAULT_TIMEOUT),
                        intakeSubsystem.transfer(),
                        new RotateDistance(carouselSubsystem, -CAROUSEL_STEP, transferSpeed),
                        new WaitUntilCommand(() -> DischargeCommands.AutomaticAiming.atCloseSpeed),
                        new RotateDistance(carouselSubsystem, -CAROUSEL_STEP * 2, transferSpeed),
                        new WaitUntilCommand(() -> DischargeCommands.AutomaticAiming.atCloseSpeed),
                        new RotateDistance(carouselSubsystem, -CAROUSEL_FULL, transferSpeed)
                ));
            }}, () -> getSequence(carouselSubsystem));
        }

        private static Sequence getSequence(CarouselSubsystem carouselSubsystem) {
            int wanted = (3 - SavedValues.currentCount % 3 + SavedValues.startMotif) % 3;
            int current = carouselSubsystem.getGreenPlacement();
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

        public Discharge(CarouselSubsystem carouselSubsystem) {
            super(new HashMap<Object, Command>() {{
                put(Sequence.CLOSE, new SequentialCommandGroup(
                        new InstantCommand(() -> DischargeSubsystem.shooting = false),
                        new WaitUntilCommand(() -> DischargeCommands.AutomaticAiming.atSpeed),
                        new RotateDistance(carouselSubsystem, 2.2, 1)
                ));
                put(Sequence.MIDDLE, new SequentialCommandGroup(

                        new RotateDistance(carouselSubsystem, 2, 0.95),
                        new WaitCommand(80),
                        new RotateDistance(carouselSubsystem, 1.2, 0.65)));
                put(Sequence.FAR, new SequentialCommandGroup(
                        new InstantCommand(() -> DischargeSubsystem.shooting = false),
                        new WaitUntilCommand(() -> DischargeCommands.AutomaticAiming.atCloseSpeed),
                        new InstantCommand(() -> DischargeSubsystem.shooting = true),
                        new RotateDistance(carouselSubsystem, 2.2, 1),
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