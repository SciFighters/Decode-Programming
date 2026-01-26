package org.firstinspires.ftc.teamcode.decodeCommands;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.ConditionalCommand;
import com.seattlesolvers.solverslib.command.SelectCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.decodeOpModes.testers.AutomaticShootingTuner;
import org.firstinspires.ftc.teamcode.decodeSubsystems.AutoShooter;
import org.firstinspires.ftc.teamcode.decodeSubsystems.CarouselSubsystem;
import org.firstinspires.ftc.teamcode.decodeSubsystems.DischargeSubsystem;
import org.firstinspires.ftc.teamcode.decodeSubsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.decodeSubsystems.Motif;
import org.firstinspires.ftc.teamcode.decodeSubsystems.SavedValues;

import java.util.HashMap;
import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public class CarouselCommands {

    public static class Manual extends CommandBase {
        private final CarouselSubsystem carouselSubsystem;
        private final Supplier<Double> power;

        public Manual(CarouselSubsystem carouselSubsystem, Supplier<Double> power) {
            this.carouselSubsystem = carouselSubsystem;
            this.power = power;
            addRequirements(carouselSubsystem);
        }

        @Override
        public void execute() {
            carouselSubsystem.setSpinPower(power.get());
        }

    }

    public static class MoveToPos extends CommandBase {
        private final CarouselSubsystem carouselSubsystem;
        double targetPos;
        double currentPos;
        double kp = 0.1;
        int tolerance = 50;

        public MoveToPos(CarouselSubsystem carouselSubsystem, double pos) {
            this.carouselSubsystem = carouselSubsystem;
            this.targetPos = pos;
        }

        @Override
        public void execute() {
            currentPos = carouselSubsystem.getPosition();
            double error = targetPos - currentPos;
            double power = kp * error;
            carouselSubsystem.setSpinPower(power);
        }

        @Override
        public boolean isFinished() {
            return Math.abs(currentPos - targetPos) < tolerance;
        }


        @Override
        public void end(boolean interrupted) {
            carouselSubsystem.setSpinPower(0);
        }
    }

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

    public static class ThirdOfSpin extends CommandBase {
        private final CarouselSubsystem carouselSubsystem;
        double targetPos;
        double currentPos;
        int tolerance = 50;
        double kp = 0.1;

        public ThirdOfSpin(CarouselSubsystem carouselSubsystem) {
            this.carouselSubsystem = carouselSubsystem;

            targetPos = carouselSubsystem.getPosition() + carouselSubsystem.spinConversion;
        }

        @Override
        public void execute() {
            currentPos = carouselSubsystem.getPosition();
            double error = targetPos - currentPos;
            double power = kp * error;
            carouselSubsystem.setSpinPower(power);
        }

        @Override
        public boolean isFinished() {
            return Math.abs(currentPos - targetPos) < tolerance;
        }

        @Override
        public void end(boolean interrupted) {
            carouselSubsystem.setSpinPower(0);
        }
    }

    public static class GreenBallToSensor extends CommandBase {
        private final CarouselSubsystem carouselSubsystem;
        private double targetPos;
        double currentPos;
        int tolerance = 50;
        double kp = 0.1;
        boolean isGreen = false;
        int moveCount = 0;

        public GreenBallToSensor(CarouselSubsystem carouselSubsystem) {
            this.carouselSubsystem = carouselSubsystem;
            addRequirements(carouselSubsystem);
        }

        @Override
        public void initialize() {
            targetPos = carouselSubsystem.getPosition();
        }

        @Override
        public void execute() {
            currentPos = carouselSubsystem.getPosition();
//            if (carouselSubsystem.colorIdentifier() == CarouselSubsystem.SensorColors.Green) {
//                isGreen = true;
//            }
            if (Math.abs(targetPos - currentPos) < tolerance && moveCount < 3) {
                targetPos += carouselSubsystem.spinConversion;
                moveCount++;
            }

            double error = targetPos - currentPos;
            double power = kp * error;
            carouselSubsystem.setSpinPower(power);
        }

        @Override
        public boolean isFinished() {
            return isGreen || moveCount >= 3;
        }

        @Override
        public void end(boolean interrupted) {
            carouselSubsystem.setSpinPower(0);
        }
    }


    public static class SortByMotif extends CommandBase {
        private final CarouselSubsystem carouselSubsystem;
        private final Motif motif;
        private int steps = 0;
        private double targetPos;
        double currentPos;
        int tolerance = 5;
        double kp = 0.01;

        public SortByMotif(Motif motif, CarouselSubsystem carouselSubsystem) {
            this.carouselSubsystem = carouselSubsystem;
            this.motif = motif;
            addRequirements(carouselSubsystem);
        }

        @Override
        public void initialize() {
            int placement = 1;//carouselSubsystem.getGreenPlacement()

            switch (motif) {
                case PGP:
                    steps = (carouselSubsystem.getGreenPlacement() + 2) % 3;
                    break;
                case GPP:
                    steps = carouselSubsystem.getGreenPlacement() % 3;
                    break;
                case PPG:
                    steps = (carouselSubsystem.getGreenPlacement() + 1) % 3;
                    break;
            }
            targetPos = (placement == -1) ? carouselSubsystem.getPosition() : (carouselSubsystem.getPosition() + steps * carouselSubsystem.spinConversion);
        }


        @Override
        public void execute() {
            currentPos = carouselSubsystem.getPosition();
            double error = targetPos - currentPos;
            double power = kp * error;
            carouselSubsystem.setSpinPower(power);
        }

        @Override
        public boolean isFinished() {
            return Math.abs(currentPos - targetPos) < tolerance;
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
            targetPos = (carouselSubsystem.getPosition() + distance * carouselSubsystem.spinConversion);
        }

        @Override
        public void execute() {
            if (carouselSubsystem.getCurrent() > 6) {
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

    public static class SafeSlide extends CommandBase {
        private final CarouselSubsystem carouselSubsystem;
        private final IntakeSubsystem intakeSubsystem;
        private double targetPos;
        double power;
        double distance;
        ElapsedTime time;
        double startTime = -1;
        boolean exit = false;

        public SafeSlide(CarouselSubsystem carouselSubsystem, IntakeSubsystem intakeSubsystem, double distance, double power) {//distance in thirds
            this.carouselSubsystem = carouselSubsystem;
            this.intakeSubsystem = intakeSubsystem;
            this.power = power;
            this.distance = distance;
            time = new ElapsedTime();
            addRequirements(carouselSubsystem);
        }

        @Override
        public void initialize() {
            exit = false;
            time.reset();
            targetPos = (carouselSubsystem.getPosition() + distance * carouselSubsystem.spinConversion);
        }

        @Override
        public void execute() {
            if (carouselSubsystem.getCurrent() > 5) {
                if(exit){
                    carouselSubsystem.setSpinPower(-power);
                }
                if(startTime == -1){
                    startTime = time.seconds();
                }
//                carouselSubsystem.setSpinPower(-power);
                if(startTime != 0 && time.seconds() - startTime > 0.6){
                    carouselSubsystem.setSpinPower(-power);
                    intakeSubsystem.setPower(-0.7);
                    intakeSubsystem.setPosition(1);
                    exit = true;
                }
            } else {
                carouselSubsystem.setSpinPower(power);
                startTime = -1;
            }
        }

        @Override
        public boolean isFinished() {
            return carouselSubsystem.getPosition()  > targetPos;
        }

        @Override
        public void end(boolean interrupted) {
            carouselSubsystem.setSpinPower(0);
//            intakeSubsystem.setPower(1);
//            intakeSubsystem.setPosition(0);
        }
    }

//    public static class SmartDischarge extends SelectCommand {
//        private static final double transferSpeed = 0.7, travelSpeed = 1;
//
//        enum Sequence {
//            ONE,//0,1,2
//            TWO,//2,1,0
//            THREE,//2,0,1
//            FOUR,//0,2,1
//            NONE
//        }
//
//        public SmartDischarge(CarouselSubsystem carouselSubsystem, IntakeSubsystem intakeSubsystem, int wanted) {
//
//            super(new HashMap<Object, Command>() {{
//                put(Sequence.ONE,);
//                put(Sequence.TWO,);
//                put(Sequence.THREE,);
//                put(Sequence.FOUR,);
//            }}, () -> getSequence(carouselSubsystem,wanted));
//        }
//        private static Sequence getSequence(CarouselSubsystem carouselSubsystem, int wanted){
//            int current
//        }
//
//
//    }

    public static class Discharge extends SelectCommand {
        private static final double transferSpeed = 0.7, travelSpeed = 1;
        enum Sequence {
            CLOSE,
            MIDDLE,
            FAR

        }

        public Discharge(CarouselSubsystem carouselSubsystem, IntakeSubsystem intakeSubsystem) {
            super(new HashMap<Object, Command>(){{
                put(Sequence.CLOSE, new SequentialCommandGroup(
                            new WaitUntilCommand(() -> DischargeCommands.AutomaticAiming.atCloseSpeed),

                            new SlideDistance(carouselSubsystem, 3.2, 1)
                            /*new SlideDistance(carouselSubsystem, 1.2, 0.65)*/));
                put(Sequence.MIDDLE,new SequentialCommandGroup(
                        new WaitUntilCommand(() -> DischargeCommands.AutomaticAiming.atCloseSpeed),

                        new SlideDistance(carouselSubsystem, 2, 0.95),
                        new WaitCommand(80),
                        new SlideDistance(carouselSubsystem, 1.2, 0.65)));
                put(Sequence.FAR, new SequentialCommandGroup(
                        new WaitUntilCommand(() -> DischargeCommands.AutomaticAiming.atSpeed),
                        new IntakeCommands.TransferState(intakeSubsystem),
                        new SlideDistance(carouselSubsystem, 0.8 - carouselSubsystem.getCarouselDistance(), transferSpeed),

                        new SlideDistance(carouselSubsystem, 0.12, transferSpeed),
                        new WaitCommand(100),
                        new WaitUntilCommand(() -> DischargeCommands.AutomaticAiming.atSpeed),
                        new SlideDistance(carouselSubsystem, 0.6, transferSpeed),

                        new WaitCommand(50),
                        new WaitUntilCommand(() -> DischargeCommands.AutomaticAiming.atSpeed),
                        new SlideDistance(carouselSubsystem, 1.2, 0.5)));


                    }},() -> {
                double distance = AutoShooter.getGoalDistance(SavedValues.position, SavedValues.teamColor);
                if(distance < 75){
                    return Sequence.CLOSE;
                } else if (distance < 105) {
                    return Sequence.MIDDLE;
                }
                return Sequence.FAR;
            });
        }
    }

    public static class WaitForFullCarousel extends CommandBase {
        private final CarouselSubsystem carouselSubsystem;
        private boolean slot1 = false;
        private boolean slot2 = false;
        private boolean slot3 = false;
        private double nextCheckPos;
        int currSlot;

        public WaitForFullCarousel(CarouselSubsystem carouselSubsystem) {
            this.carouselSubsystem = carouselSubsystem;
            addRequirements(carouselSubsystem);
        }

        @Override
        public void initialize() {
            nextCheckPos = carouselSubsystem.getPosition() + carouselSubsystem.spinConversion;
            double spinPower = 0.2;
            carouselSubsystem.setSpinPower(spinPower);
        }

        @Override
        public void execute() {
            double currentPos = carouselSubsystem.getPosition();

            if (currentPos >= nextCheckPos) {
//                CarouselSubsystem.SensorColors color = carouselSubsystem.colorIdentifier();
//                nextCheckPos += carouselSubsystem.spinConversion;
//
//                if (color == CarouselSubsystem.SensorColors.Green
//                        || color == CarouselSubsystem.SensorColors.Purple) {
//
//                    currSlot =  (int) (currentPos / carouselSubsystem.spinConversion);
//                    switch (currSlot) {
//                        case 1:
//                            slot1 = true;
//                            break;
//                        case 2:
//                            slot2 = true;
//                            break;
//                        case 3:
//                            slot3 = true;
//                            break;
//                    }
//                }
            }
        }

        @Override
        public boolean isFinished() {
            return slot1 && slot2 && slot3;
        }

        @Override
        public void end(boolean interrupted) {
            carouselSubsystem.setSpinPower(0);
        }
    }


}
