package org.firstinspires.ftc.teamcode.needle.commands;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.Subsystem;
//nuh uh
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.decodeSubsystems.AutoShooter;

import java.util.HashSet;
import java.util.Set;
import java.util.function.Supplier;

public class MecanumCommands {

    public static class Drive extends CommandBase {
        MecanumDrive mecanumDrive;
        Supplier<Double> x, y, r, boost;
        AutoShooter.TeamColor teamColor;

        public Drive(MecanumDrive mecanumDrive, Supplier<Double> x, Supplier<Double> y, Supplier<Double> r) {
            this.mecanumDrive = mecanumDrive;
            this.x = x;
            this.y = y;
            this.r = r;
            this.boost = () -> 1.0;
            addRequirements(mecanumDrive);
        }
        public Drive(MecanumDrive mecanumDrive, Supplier<Double> x, Supplier<Double> y, Supplier<Double> r, Supplier<Double> boost,AutoShooter.TeamColor teamColor) {

            this.mecanumDrive = mecanumDrive;
            this.x = x;
            this.y = y;
            this.r = r;
            this.boost = boost;
            this.teamColor = teamColor;
            addRequirements(mecanumDrive);
        }

        @Override
        public void initialize() {
            mecanumDrive.lazyImu.get().resetYaw();
        }

        @Override
        public void execute() {
            double blue = (teamColor == AutoShooter.TeamColor.BLUE) ? Math.PI : 0;
            com.seattlesolvers.solverslib.geometry.Vector2d vector = new com.seattlesolvers.solverslib.geometry.Vector2d(
                    -x.get() * boost.get(), y.get() * boost.get()).rotateBy(Math.toDegrees(-mecanumDrive.localizer.getPose().heading.toDouble() - Math.PI / 2 + blue));
            Vector2d vector2d = new Vector2d(vector.getX(), vector.getY());
            mecanumDrive.setDrivePowers(new PoseVelocity2d(new Vector2d(vector2d.x, vector2d.y), -r.get() * boost.get()));
        }
    }
    public static class Aim extends CommandBase{
        MecanumDrive mecanumDrive;
        AutoShooter.TeamColor teamColor;
        final double kp = 0.6;
        public Aim(MecanumDrive mecanumDrive, AutoShooter.TeamColor teamColor){
            this.mecanumDrive = mecanumDrive;
            this.teamColor = teamColor;
            addRequirements(mecanumDrive);
        }

        @Override
        public void execute() {
            double launchAngle = ((AutoShooter.getLaunchAngle(new Pose2d(mecanumDrive.localizer.getPose().position, 0), teamColor) + 360) % 360) / 180 * Math.PI;
            double error = launchAngle - mecanumDrive.localizer.getPose().heading.toDouble();
            mecanumDrive.setDrivePowers(new PoseVelocity2d(new Vector2d(0,0),error * kp + Math.signum(error) * 0.075));

        }
    }

}
