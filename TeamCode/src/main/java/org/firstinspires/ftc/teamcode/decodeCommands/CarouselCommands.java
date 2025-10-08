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
        int currentPos;
        double kp = 0.1;
        int tolerance = 50;

        public MoveToPos(CarouselSubsystem carouselSubsystem, double pos) {
            this.carouselSubsystem = carouselSubsystem;
            targetPos = pos;
        }

        @Override
        public void execute() {
            currentPos = (int) carouselSubsystem.getPosition();
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
        int targetPos;
        int currentPos;
        int tolerance = 50;
        double kp = 0.1;
        public ThirdOfSpin(CarouselSubsystem carouselSubsystem) {
            this.carouselSubsystem = carouselSubsystem;

            targetPos = (int) (carouselSubsystem.getPosition() + carouselSubsystem.spinConversion);
        }

        @Override
        public void execute() {
            currentPos = (int) carouselSubsystem.getPosition();
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

    public static class GreenBallToBackSlot extends CommandBase {
        private final CarouselSubsystem carouselSubsystem;
        private int targetPos;
        int currentPos;
        int tolerance = 50;
        double kp = 0.1;
        public GreenBallToBackSlot(CarouselSubsystem carouselSubsystem) {
            this.carouselSubsystem = carouselSubsystem;
            addRequirements(carouselSubsystem);
        }

        @Override
        public void initialize() {
            // is there a green ball
            if (carouselSubsystem.colorIdentifier(carouselSubsystem.colorSensor) == CarouselSubsystem.SensorColors.Green) {
                // moves the carousel 1 slot (the location of the color sensor is to the left of the back slot from top view)
                targetPos = (int) (carouselSubsystem.getPosition() + carouselSubsystem.spinConversion);
            }
        }

        @Override
        public void execute() {
            currentPos = (int) carouselSubsystem.getPosition();
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

    public static class SortByMotif extends CommandBase {
        private final CarouselSubsystem carouselSubsystem;
        private final Motif motif;
        private int steps;
        private int targetPos;
        int currentPos;
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
                case GPP:
                    steps = 1;
                    break;
                case PPG:
                    steps = -1;
                    break;
                default: // for PGP case
                    steps = 0;
                    break;
            }
            targetPos = (int) (carouselSubsystem.getPosition() + steps * carouselSubsystem.spinConversion);
        }


        @Override
        public void execute() {
            currentPos = (int) carouselSubsystem.getPosition();
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
        private int targetPos;
        int currentPos;
        int tolerance = 50;
        double kp = 0.1;
        public Discharge(CarouselSubsystem carouselSubsystem) {
            this.carouselSubsystem = carouselSubsystem;
            addRequirements(carouselSubsystem);
        }

        @Override
        public void initialize() {
            // Set target position for a full rotation
            targetPos = (int) (carouselSubsystem.getPosition() + 3 * carouselSubsystem.spinConversion);
        }
        @Override
        public void execute() {
            currentPos = (int) carouselSubsystem.getPosition();
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
}
