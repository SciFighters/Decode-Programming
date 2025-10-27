package org.firstinspires.ftc.teamcode.decodeCommands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.decodeSubsystems.CarouselSubsystem;
import org.firstinspires.ftc.teamcode.decodeSubsystems.Motif;

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
            if (carouselSubsystem.colorIdentifier() == CarouselSubsystem.SensorColors.Green) {
                isGreen = true;
            }
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
        private int steps;
        private double targetPos;
        double currentPos;
        int tolerance = 50;
        double kp = 0.1;

        public SortByMotif(Motif motif, int steps, CarouselSubsystem carouselSubsystem) {
            this.carouselSubsystem = carouselSubsystem;
            this.motif = motif;
            this.steps = steps;
            addRequirements(carouselSubsystem);
        }

        @Override
        public void initialize() {
            targetPos = carouselSubsystem.getPosition();
        }

        @Override
        public void execute() {
            currentPos = carouselSubsystem.getPosition();
            if (carouselSubsystem.colorIdentifier() == CarouselSubsystem.SensorColors.Green) {
                isGreen = true;
            }
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
        private int steps;
        private double targetPos;
        double currentPos;
        int tolerance = 50;
        double kp = 0.1;

        public SortByMotif(Motif motif, int steps, CarouselSubsystem carouselSubsystem) {
            this.carouselSubsystem = carouselSubsystem;
            this.motif = motif;
            this.steps = steps;
            addRequirements(carouselSubsystem);
        }

        @Override
        public void initialize() {
            switch (motif) {
                case PGP:
                    steps = 1;
                    break;
                case GPP:
                    steps = -1;
                    break;
                default: // for PPG case
                    steps = 0;
                    break;
            }
            targetPos = (carouselSubsystem.getPosition() + steps * carouselSubsystem.spinConversion);
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


    public static class Discharge extends CommandBase {
        private final CarouselSubsystem carouselSubsystem;
        private double targetPos;
        double currentPos;
        double power = 0.6;

        public Discharge(CarouselSubsystem carouselSubsystem) {
            this.carouselSubsystem = carouselSubsystem;
            addRequirements(carouselSubsystem);
        }

        @Override
        public void initialize() {
            // Set target position for a full rotation
            double angle = carouselSubsystem.getAngle();
            if(100 < angle && angle < 140){
                targetPos = (carouselSubsystem.getPosition() + 4 * carouselSubsystem.spinConversion);
            }
            else if(220 < angle && angle < 260){
                targetPos = (carouselSubsystem.getPosition() + 5 * carouselSubsystem.spinConversion);

            }
            else if(340 < angle || angle < 20){
                targetPos = (carouselSubsystem.getPosition() + 3 * carouselSubsystem.spinConversion);
            }
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
        private double nextCheckPos ;
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
                CarouselSubsystem.SensorColors color = carouselSubsystem.colorIdentifier();
                nextCheckPos += carouselSubsystem.spinConversion;

                if (color == CarouselSubsystem.SensorColors.Green
                        || color == CarouselSubsystem.SensorColors.Purple) {

                    currSlot =  (int) (currentPos / carouselSubsystem.spinConversion);
                    switch (currSlot) {
                        case 1:
                            slot1 = true;
                            break;
                        case 2:
                            slot2 = true;
                            break;
                        case 3:
                            slot3 = true;
                            break;
                    }
                }
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
