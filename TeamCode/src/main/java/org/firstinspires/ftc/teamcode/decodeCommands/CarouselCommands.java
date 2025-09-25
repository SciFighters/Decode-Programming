package org.firstinspires.ftc.teamcode.decodeCommands;

import com.seattlesolvers.solverslib.command.CommandBase;
import org.firstinspires.ftc.teamcode.decodeSubsystems.CarouselSubsystem;

public class CarouselCommands {

    public static class MoveToPos extends CommandBase {
        private final CarouselSubsystem carouselSubsystem;

        int currentPos;
        double tagetPos;
        double kp = 0.1;

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
            return Math.abs(currentPos - tagetPos) < 50;
        }

        @Override
        public void end(boolean interrupted) {
            carouselSubsystem.stopMotor();
        }
    }

    public static class ThirdOfSpin extends CommandBase {
        private final CarouselSubsystem carouselSubsystem;
        double power;
        int currentPos;
        int tagetPos;
        double kp = 0.1;

        public ThirdOfSpin(CarouselSubsystem carouselSubsystem) {
            this.carouselSubsystem = carouselSubsystem;

            tagetPos = (int) (carouselSubsystem.getPosition() + carouselSubsystem.spinConversion);
        }

        @Override
        public void initialize() {
            int currentPos = (int) carouselSubsystem.getPosition();

            double error = tagetPos - currentPos;
            double power = kp * error;

            carouselSubsystem.setSpinPower(power);
        }

        @Override
        public boolean isFinished() {
            return Math.abs(currentPos - tagetPos) < 50;
        }
    }
}
