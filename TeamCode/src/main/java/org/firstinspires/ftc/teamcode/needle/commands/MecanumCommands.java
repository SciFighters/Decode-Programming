package org.firstinspires.ftc.teamcode.needle.commands;

import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.teamcode.actions.ActionBase;
import org.firstinspires.ftc.teamcode.MecanumDrive;

import java.util.function.Supplier;

public class MecanumCommands {
    public static class Drive extends ActionBase {
        MecanumDrive mecanumDrive;
        Supplier<Double> x, y, r;

        public Drive(MecanumDrive mecanumDrive, Supplier<Double> x, Supplier<Double> y, Supplier<Double> r) {
            this.mecanumDrive = mecanumDrive;
            this.x = x;
            this.y = y;
            this.r = r;
        }

        @Override
        public void execute() {
            com.arcrobotics.ftclib.geometry.Vector2d vector
                    = new com.arcrobotics.ftclib.geometry.Vector2d(
                    x.get(), y.get()).rotateBy(-mecanumDrive.localizer.getPose().heading.toDouble());
            Vector2d vector2d = new Vector2d(vector.getX(), vector.getY());
            mecanumDrive.setDrivePowers(new PoseVelocity2d(vector2d,r.get()));
        }
    }
}
