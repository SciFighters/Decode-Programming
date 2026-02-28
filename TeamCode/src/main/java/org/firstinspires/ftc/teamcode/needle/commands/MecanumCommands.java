package org.firstinspires.ftc.teamcode.needle.commands;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.Subsystem;
//nuh uh
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.decodeCommands.DischargeCommands;
import org.firstinspires.ftc.teamcode.decodeSubsystems.AutoShooter;

import java.util.HashSet;
import java.util.Set;
import java.util.function.Supplier;

public class MecanumCommands {

    public static class Drive extends CommandBase {
        MecanumDrive mecanumDrive;
        Supplier<Double> x, y, r, boost;
        AutoShooter.TeamColor teamColor;
        double lastX = 0,lastY = 0;
        double blue;
        Vector2d holdPos = new Vector2d(0,0);
        public static double kp = 0.01, kd = -0.026;

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
            blue = (teamColor == AutoShooter.TeamColor.BLUE) ? Math.PI : 0;
        }

        @Override
        public void execute() {
            double currentX = x.get();
            double currentY = y.get();
            if((currentX == 0 && currentY == 0) && DischargeCommands.AutomaticAiming.shooting && AutoShooter.canLaunch(mecanumDrive.localizer.getPose())){
                Vector2d pos = mecanumDrive.localizer.getPose().position;
                Vector2d velocity = mecanumDrive.localizer.update().linearVel;
                if((lastX != 0 || lastY != 0) || (holdPos.x == 0 && holdPos.y == 0)){
                    holdPos = pos;
                }
                Vector2d delta = new Vector2d(holdPos.x - pos.x, holdPos.y - pos.y);
                com.seattlesolvers.solverslib.geometry.Vector2d vector = new com.seattlesolvers.solverslib.geometry.Vector2d
                        (delta.x * kp + velocity.x * kd,delta.y * kp + velocity.y * kd).rotateBy(-mecanumDrive.localizer.getPose().heading.toDouble());
                Vector2d vector2d = new Vector2d(vector.getX(), vector.getY());
                mecanumDrive.setDrivePowers(new PoseVelocity2d(new Vector2d(vector2d.x, vector2d.y), -r.get() * boost.get()));
            }else{
            com.seattlesolvers.solverslib.geometry.Vector2d vector = new com.seattlesolvers.solverslib.geometry.Vector2d(
                    -currentX * boost.get(), currentY * boost.get()).rotateBy(Math.toDegrees(-mecanumDrive.localizer.getPose().heading.toDouble() - Math.PI / 2 + blue));
            Vector2d vector2d = new Vector2d(vector.getX(), vector.getY());
            mecanumDrive.setDrivePowers(new PoseVelocity2d(new Vector2d(vector2d.x, vector2d.y), -r.get() * boost.get()));
            }
            lastX = currentX;
            lastY = currentY;
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
