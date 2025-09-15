package org.firstinspires.ftc.teamcode.needle.commands;

import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.Subsystem;
//nuh uh
import org.firstinspires.ftc.teamcode.MecanumDrive;

import java.util.HashSet;
import java.util.Set;
import java.util.function.Supplier;

public class MecanumCommands {

    public static class Drive extends CommandBase {
        MecanumDrive mecanumDrive;
        Supplier<Double> x, y, r, boost;

        public Drive(MecanumDrive mecanumDrive, Supplier<Double> x, Supplier<Double> y, Supplier<Double> r) {
            this.mecanumDrive = mecanumDrive;
            this.x = x;
            this.y = y;
            this.r = r;
            this.boost = () -> 1.0;
            addRequirements(mecanumDrive);
        }
        public Drive(MecanumDrive mecanumDrive, Supplier<Double> x, Supplier<Double> y, Supplier<Double> r, Supplier<Double> boost) {
            this.mecanumDrive = mecanumDrive;
            this.x = x;
            this.y = y;
            this.r = r;
            this.boost = boost;
            addRequirements(mecanumDrive);
        }

        @Override
        public void execute() {
            com.seattlesolvers.solverslib.geometry.Vector2d vector = new com.seattlesolvers.solverslib.geometry.Vector2d(
                    x.get() * boost.get(), y.get() * boost.get()).rotateBy(Math.toDegrees(-mecanumDrive.localizer.getPose().heading.toDouble() + Math.PI/2));
            Vector2d vector2d = new Vector2d(vector.getX(), vector.getY());
            mecanumDrive.setDrivePowers(new PoseVelocity2d(vector2d,-r.get() * boost.get()));
        }
    }

}
