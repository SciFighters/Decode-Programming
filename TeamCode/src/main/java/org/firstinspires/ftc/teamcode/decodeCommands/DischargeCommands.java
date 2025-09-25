package org.firstinspires.ftc.teamcode.decodeCommands;

import com.acmerobotics.dashboard.config.Config;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.decodeSubsystems.DischargeSubsystem;

import java.util.function.Supplier;
@Config
public class DischargeCommands {
    public static class DischargeManual extends CommandBase {
        DischargeSubsystem dischargeSubsystem;
        Supplier<Double> flyWheelPower;
        Supplier<Double> turretPower;
        double rampDegree;
        public DischargeManual(DischargeSubsystem dischargeSubsystem, Supplier<Double> flyWheelPower, Supplier<Double> turretPower, double rampDegree) {
            this.dischargeSubsystem = dischargeSubsystem;
            this.flyWheelPower = flyWheelPower;
            this.turretPower = turretPower;
            this.rampDegree = rampDegree;
            addRequirements(dischargeSubsystem);
        }

        @Override
        public void execute() {
            dischargeSubsystem.setFlyWheelPower(flyWheelPower.get());
            dischargeSubsystem.setTurretPower(turretPower.get());
        }
    }
    public static class GoTo extends CommandBase {
        DischargeSubsystem dischargeSubsystem;
        double turretPower;
        double flyWheelPower;
        double rampDegree;
        double wantedPos;
        static final double kp = 0.01;

        public GoTo(DischargeSubsystem dischargeSubsystem,  double flyWheelPower, double turretPos, double rampDegree){
            this.dischargeSubsystem = dischargeSubsystem;
            this.flyWheelPower = flyWheelPower;
            this.rampDegree = rampDegree;
            wantedPos = turretPos;
            addRequirements(dischargeSubsystem);
        }

        @Override
        public void execute() {
            double power = (wantedPos - dischargeSubsystem.getTurretPosition()) * kp;
            dischargeSubsystem.setTurretPower(power);
            dischargeSubsystem.setFlyWheelPower(flyWheelPower);
            dischargeSubsystem.setRampDegree(rampDegree);


        }
    }
}
