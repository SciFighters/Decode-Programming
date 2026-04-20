package org.firstinspires.ftc.teamcode.needle.commands;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.decodeCommands.DischargeCommands;
import org.firstinspires.ftc.teamcode.decodeSubsystems.AutoShooter;

import java.util.function.Supplier;

public class MecanumCommands {

    public static class Drive extends CommandBase {
        public static double kp = 0, kd = -0.026;
        MecanumDrive mecanumDrive;
        Supplier<Double> x, y, r, boost;
        AutoShooter.TeamColor teamColor;
        double lastX = 0, lastY = 0;
        double blue;
        Vector2d holdPos = new Vector2d(0, 0);

        public Drive(MecanumDrive mecanumDrive, Supplier<Double> x, Supplier<Double> y, Supplier<Double> r) {
            this.mecanumDrive = mecanumDrive;
            this.x = x;
            this.y = y;
            this.r = r;
            this.boost = () -> 1.0;
            addRequirements(mecanumDrive);
        }

        public Drive(MecanumDrive mecanumDrive, Supplier<Double> x, Supplier<Double> y, Supplier<Double> r, Supplier<Double> boost, AutoShooter.TeamColor teamColor) {

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
            blue = (teamColor == AutoShooter.TeamColor.BLUE) ? Math.PI : 0;
        }

        @Override
        public void execute() {
            double currentX = x.get();
            double currentY = y.get();
            if ((currentX == 0 && currentY == 0) && DischargeCommands.AutomaticAiming.shooting && AutoShooter.canLaunch(mecanumDrive.localizer.getPose()) && false) {
                Vector2d pos = mecanumDrive.localizer.getPose().position;
                Vector2d velocity = mecanumDrive.localizer.update().linearVel;
                if ((lastX != 0 || lastY != 0) || (holdPos.x == 0 && holdPos.y == 0)) {
                    holdPos = pos;
                }
                Vector2d delta = new Vector2d(holdPos.x - pos.x, holdPos.y - pos.y);
                com.seattlesolvers.solverslib.geometry.Vector2d vector = new com.seattlesolvers.solverslib.geometry.Vector2d
                        (delta.x * kp + velocity.x * kd, delta.y * kp + velocity.y * kd).rotateBy(-mecanumDrive.localizer.getPose().heading.toDouble());
                Vector2d vector2d = new Vector2d(vector.getX(), vector.getY());
                mecanumDrive.setDrivePowers(new PoseVelocity2d(new Vector2d(vector2d.x, vector2d.y), -r.get() * boost.get()));
            } else {
                com.seattlesolvers.solverslib.geometry.Vector2d vector = new com.seattlesolvers.solverslib.geometry.Vector2d(
                        -currentX * boost.get(), currentY * boost.get()).rotateBy(Math.toDegrees(-mecanumDrive.localizer.getPose().heading.toDouble() - Math.PI / 2 + blue));
                Vector2d vector2d = new Vector2d(vector.getX(), vector.getY());
                mecanumDrive.setDrivePowers(new PoseVelocity2d(new Vector2d(vector2d.x, vector2d.y), -r.get() * boost.get()));
            }
            lastX = currentX;
            lastY = currentY;
        }
    }

    @Config
    public static class Aim extends CommandBase {
        public static double kp = 0.5, kd = -0.001, ki = 0, kf = 0.1, deadZone = 2;
        MecanumDrive mecanumDrive;
        AutoShooter.TeamColor teamColor;
        double currentError = 0, lastError = 0;
        double integral = 0, derivative;
        Supplier<Double> x, y, boost;
        ElapsedTime time;
        double blue, lastTime;
        double angle;

        public Aim(MecanumDrive mecanumDrive, Supplier<Double> x, Supplier<Double> y, Supplier<Double> boost,double angle, AutoShooter.TeamColor teamColor) {
            this.mecanumDrive = mecanumDrive;
            this.teamColor = teamColor;
            addRequirements(mecanumDrive);
            this.x = x;
            this.y = y;
            this.boost = boost;
            this.angle = angle;
        }

        @Override
        public void initialize() {
            blue = (teamColor == AutoShooter.TeamColor.BLUE) ? Math.PI : 0;
            time = new ElapsedTime();
            time.reset();
            lastTime = time.seconds();
        }

        @Override
        public void execute() {
            double currentTime = time.seconds();
            double deltaTIme = currentTime - lastTime;
            double currentX = x.get();
            double currentY = y.get();
            double wantedAngle = (teamColor == AutoShooter.TeamColor.BLUE) ? angle : -angle;
            double currentAngle = mecanumDrive.localizer.getPose().heading.toDouble();
            currentError = AutoShooter.normalizeRadianError(wantedAngle - currentAngle);
            if(Math.abs(currentError) < deadZone / 180 * Math.PI){
                currentError = 0;
            }

            integral += currentError * deltaTIme;
            derivative = (currentError - lastError) / deltaTIme;
            com.seattlesolvers.solverslib.geometry.Vector2d vector = new com.seattlesolvers.solverslib.geometry.Vector2d(
                    -currentX * boost.get(), currentY * boost.get()).rotateBy(Math.toDegrees(-mecanumDrive.localizer.getPose().heading.toDouble() - Math.PI / 2 + blue));
            Vector2d vector2d = new Vector2d(vector.getX(), vector.getY());
            mecanumDrive.setDrivePowers(new PoseVelocity2d(new Vector2d(vector2d.x, vector2d.y), currentError * kp + integral * ki + derivative * kd + Math.signum(currentError) * kf));
            lastTime = currentTime;
            lastError = currentError;
        }


    }

}
