package org.firstinspires.ftc.teamcode.decodeCommands;

import com.seattlesolvers.solverslib.command.CommandBase;
import org.firstinspires.ftc.teamcode.decodeSubsystems.CarouselSubsystem;

public class CarouselCommands {

    public static class MoveCarouselByTicks extends CommandBase {
        private final CarouselSubsystem carouselSubsystem;
        private final double power;
        int mul;
        public MoveCarouselByTicks(CarouselSubsystem carouselSubsystem, double power, boolean direction) {
            if (direction) { mul = 1; }
            else { mul = -1; }

            this.carouselSubsystem = carouselSubsystem;
            this.power = power * mul;
            addRequirements(carouselSubsystem);

        }

        @Override
        public void initialize() {
            int currentPos = (int) carouselSubsystem.getPosition();
            int targetPos = currentPos + (int) carouselSubsystem.spinConversion * mul;

            carouselSubsystem.moveToPosition(targetPos, power);
        }

        @Override
        public boolean isFinished() {
            return carouselSubsystem.atTarget();
        }

        @Override
        public void end(boolean interrupted) {
            carouselSubsystem.stopMotor();
        }
    }
}
