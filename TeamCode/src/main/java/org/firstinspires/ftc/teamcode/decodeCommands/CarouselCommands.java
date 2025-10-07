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

        int currentPos;
        double tagetPos;
        double kp = 0.1;
        int tolerance = 50;

        public MoveToPos(CarouselSubsystem carouselSubsystem, double pos) {
            this.carouselSubsystem = carouselSubsystem;
            tagetPos = pos;
        }

        @Override
        public void execute() {
            int currentPos = (int) carouselSubsystem.getPosition();

            double error = tagetPos - currentPos;
            double power = kp * error;

            carouselSubsystem.setSpinPower(power);
        }

        @Override
        public boolean isFinished() {
            return Math.abs(currentPos - tagetPos) < tolerance;
        }

        @Override
        public void end(boolean interrupted) {
            carouselSubsystem.setSpinPower(0);
        }
    }

    public static class ThirdOfSpin extends CommandBase {
        private final CarouselSubsystem carouselSubsystem;
        int currentPos;
        int tagetPos;
        double kp = 0.1;
        int tolerance = 50;

        public ThirdOfSpin(CarouselSubsystem carouselSubsystem) {
            this.carouselSubsystem = carouselSubsystem;

            tagetPos = (int) (carouselSubsystem.getPosition() + carouselSubsystem.spinConversion);
        }

        @Override
        public void execute() {
            int currentPos = (int) carouselSubsystem.getPosition();

            double error = tagetPos - currentPos;
            double power = kp * error;

            carouselSubsystem.setSpinPower(power);
        }

        @Override
        public boolean isFinished() {
            return Math.abs(currentPos - tagetPos) < tolerance;
        }

        @Override
        public void end(boolean interrupted) {
            carouselSubsystem.setSpinPower(0);
        }
    }

    public static class GreenBallToBackSlot extends CommandBase {
        private final CarouselSubsystem carouselSubsystem;
        private int targetPos;

        public GreenBallToBackSlot(CarouselSubsystem carouselSubsystem) {
            this.carouselSubsystem = carouselSubsystem;
        }

        @Override
        public void initialize() {
            // is there a green ball
            if (carouselSubsystem.colorIdentifier(carouselSubsystem.colorSensor) == CarouselSubsystem.SensorColors.Green) {
                // moves the carousel 2 slot (depending on the location of the color sensor)
                targetPos = (int) (carouselSubsystem.getPosition() + 2 * carouselSubsystem.spinConversion);
            }
        }

        @Override
        public void execute() {
            int currentPos = (int) carouselSubsystem.getPosition();
            double error = targetPos - currentPos;
            double kp = 0.1;
            double power = kp * error;

            carouselSubsystem.setSpinPower(power);
        }

        @Override
        public boolean isFinished() {
            int currentPos = (int) carouselSubsystem.getPosition();
            int tolerance = 50;
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
        private final double kp = 0.1;
        private final int tolerance = 50;

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
                case PGP:
                    steps = 0;
                    break;
                case PPG:
                    steps = -1;
                    break;
            }
            targetPos = (int) (carouselSubsystem.getPosition() + steps * carouselSubsystem.spinConversion);
        }


        @Override
        public void execute() {
            int currentPos = (int) carouselSubsystem.getPosition();
            double error = targetPos - currentPos;
            double power = Math.max(-1, Math.min(1, kp * error));
            carouselSubsystem.setSpinPower(power);
        }

        @Override
        public boolean isFinished() {
            int currentPos = (int) carouselSubsystem.getPosition();
            return Math.abs(currentPos - targetPos) < tolerance;
        }

        @Override
        public void end(boolean interrupted) {
            carouselSubsystem.setSpinPower(0);
        }
    }


    public class Discharge extends CommandBase {
        private final CarouselSubsystem carouselSubsystem;
        private int targetPos;
        private final double kp = 0.1;
        private final int tolerance = 50;

        public Discharge(CarouselSubsystem carouselSubsystem) {
            this.carouselSubsystem = carouselSubsystem;
            addRequirements(carouselSubsystem);
        }

        @Override
        public void initialize() {
            // Set target position for a full rotation (3 thirds)
            targetPos = (int) (carouselSubsystem.getPosition() + 3 * carouselSubsystem.spinConversion);
        }
        @Override
        public void execute() {
            int currentPos = (int) carouselSubsystem.getPosition();
            double error = targetPos - currentPos;
            // Clamp power to [-1, 1]
            double power = Math.max(-1, Math.min(1, kp * error));
            carouselSubsystem.setSpinPower(power);
        }

        @Override
        public boolean isFinished() {
            int currentPos = (int) carouselSubsystem.getPosition();
            return Math.abs(currentPos - targetPos) < tolerance;
        }
    }
}
