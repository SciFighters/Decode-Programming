package org.firstinspires.ftc.teamcode.decodeCommands;

import androidx.annotation.NonNull;

import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.SelectCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.decodeSubsystems.CarouselSubsystem;
import org.firstinspires.ftc.teamcode.decodeSubsystems.DischargeSubsystem;
import org.firstinspires.ftc.teamcode.decodeSubsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.decodeSubsystems.Motif;

import java.util.HashMap;
import java.util.Map;
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
        Supplier<Double> distanceSupplier;

        public SlideDistance(CarouselSubsystem carouselSubsystem, double distance, double power) {//distance in thirds
            this.carouselSubsystem = carouselSubsystem;
            this.power = power;
            this.distance = distance;
            distanceSupplier = null;
            addRequirements(carouselSubsystem);
        }

        public SlideDistance(CarouselSubsystem carouselSubsystem, Supplier<Double> distanceSupplier, double power) {//distance in thirds
            this.carouselSubsystem = carouselSubsystem;
            this.power = power;
            this.distanceSupplier = distanceSupplier;
            addRequirements(carouselSubsystem);
        }

        @Override
        public void initialize() {
            if (distanceSupplier != null) {
                distance = Math.min(distanceSupplier.get() + 0.55, -0.03);
            }
            moveDirection = (int)Math.signum(distance);
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

    public static class SmartDischarge extends SelectCommand {
        private static final double transferSpeed = 0.7, travelSpeed = 1;

        enum Position {
            MIDDLE,
            LEFT,
            RIGHT,
            NONE
        }

        public SmartDischarge(CarouselSubsystem carouselSubsystem, IntakeSubsystem intakeSubsystem) {

            super(new HashMap<Object, Command>() {{
                put(Position.MIDDLE,
                        new SequentialCommandGroup(
                                new WaitUntilCommand(() -> DischargeCommands.AutomaticAiming.atSpeed),
                                new SlideDistance(carouselSubsystem, 0.8, transferSpeed),
//                                new SlideDistance(carouselSubsystem,0.5,transferSpeed),

                                new SlideDistance(carouselSubsystem, 0.4, travelSpeed),
                                new WaitCommand(150),
                                new WaitUntilCommand(() -> DischargeCommands.AutomaticAiming.atSpeed),
                                new SlideDistance(carouselSubsystem, 1.6 - 0.8, transferSpeed),

                                new SlideDistance(carouselSubsystem, 0.6, travelSpeed),
                                new WaitCommand(50),
                                new WaitUntilCommand(() -> DischargeCommands.AutomaticAiming.atSpeed),
                                new SlideDistance(carouselSubsystem, 0.4, transferSpeed)
                        ));
                put(Position.LEFT,
                        new SequentialCommandGroup(
                                new SlideDistance(carouselSubsystem, 1.7, transferSpeed),
                                new SlideDistance(carouselSubsystem, 2.3, travelSpeed),
                                new SlideDistance(carouselSubsystem, 1, transferSpeed)
                        ));
                put(Position.RIGHT,
                        new SequentialCommandGroup(
                                new SlideDistance(carouselSubsystem, 1, transferSpeed),
                                new SlideDistance(carouselSubsystem, 1, travelSpeed),
                                new SlideDistance(carouselSubsystem, 2, transferSpeed)
                        ));
                put(Position.NONE, new SlideDistance(carouselSubsystem, 6, transferSpeed));
            }}, () -> getPosition(carouselSubsystem));
        }

        private static Position getPosition(CarouselSubsystem carouselSubsystem) {
            double angle = carouselSubsystem.getAngle();
            if (280 < angle && angle < 320) {
                return Position.LEFT;

            } else if (40 < angle && angle < 80) {
                return Position.RIGHT;

            } else if (160 < angle || angle < 200) {
                return Position.MIDDLE;
            }
            return Position.NONE;

        }
    }

    public static class Discharge extends CommandBase {
        private final CarouselSubsystem carouselSubsystem;
        private double targetPos;
        double currentPos;
        double transferPower = 0.3;
        double toNextPower = 0.75; // when it moves until the next ball

        public Discharge(CarouselSubsystem carouselSubsystem) {
            this.carouselSubsystem = carouselSubsystem;
            addRequirements(carouselSubsystem);
        }

        @Override
        public void initialize() {
            double angle = carouselSubsystem.getAngle();
            if (100 < angle && angle < 140) {
                targetPos = (carouselSubsystem.getPosition() + 6 * carouselSubsystem.spinConversion);
            } else if (220 < angle && angle < 260) {
                targetPos = (carouselSubsystem.getPosition() + 6 * carouselSubsystem.spinConversion);

            } else if (340 < angle || angle < 20) {
                targetPos = (carouselSubsystem.getPosition() + 6 * carouselSubsystem.spinConversion);
            }
        }

        @Override
        public void execute() {
            currentPos = carouselSubsystem.getPosition();
            //if (DOESNT PUSHES BALLS)
            //    carouselSubsystem.setSpinPower(toNextPower);
            //else
            carouselSubsystem.setSpinPower(transferPower);
        }

        @Override
        public boolean isFinished() {
            return currentPos > targetPos;
        }

        @Override
        public void end(boolean interrupted) {
            carouselSubsystem.setSpinPower(0);
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
