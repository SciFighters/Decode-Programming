package org.firstinspires.ftc.teamcode.decodeCommands;

import com.acmerobotics.dashboard.config.Config;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.decodeSubsystems.DischargeSubsystem;

import java.util.function.Supplier;
@Config
public class DischargeCommands{
    public static class DischargeManual extends CommandBase{
        DischargeSubsystem dischargeSubsystem;
        Supplier<Double> flyWheelPower;
        Supplier<Double> turretPower;
        Supplier<Double> rampDegree;
        public DischargeManual(DischargeSubsystem dischargeSubsystem, Supplier<Double> flyWheelPower, Supplier<Double> turretPower, Supplier<Double> rampDegree) {
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
            dischargeSubsystem.setRampDegree(rampDegree.get());
        }
    }
    public static class setState extends CommandBase{
        DischargeSubsystem dischargeSubsystem;
        double flyWheelRPM;
        double rampDegree;
        double wantedPos;
        public static boolean canShoot;//variable for knowing if you can shout, will be removed later with automatic shooting
        static final double kp = 0.01;

        public setState(DischargeSubsystem dischargeSubsystem, double flyWheelRPM, double turretPos, double rampDegree){
            this.dischargeSubsystem = dischargeSubsystem;
            this.flyWheelRPM = flyWheelRPM;
            this.rampDegree = rampDegree;
            wantedPos = turretPos;
            addRequirements(dischargeSubsystem);
        }
        @Override
        public  void initialize() {
            canShoot = false;
            dischargeSubsystem.setRampDegree(rampDegree);
        }

        @Override
        public void execute() {
            dischargeSubsystem.setFlyWheelRPM(flyWheelRPM);
            double power = (wantedPos - dischargeSubsystem.getTurretPosition()) * kp;
            dischargeSubsystem.setTurretPower(power);
            canShoot = Math. abs(dischargeSubsystem.getRPM() - flyWheelRPM) < 300;
        }

    }
    public static class SupplierGoTo extends CommandBase{
        DischargeSubsystem dischargeSubsystem;
        Supplier<Double> flyWheelPower;
        Supplier<Double> turretPower;
        Supplier<Double> rampDegree;
        double wantedPos;
        static final double kp = 0.01;
        public SupplierGoTo(DischargeSubsystem dischargeSubsystem, Supplier<Double> flyWheelPower, Supplier<Double> turretPower, Supplier<Double> rampDegree) {
            this.dischargeSubsystem = dischargeSubsystem;
            this.flyWheelPower = flyWheelPower;
            this.turretPower = turretPower;
            this.rampDegree = rampDegree;
            addRequirements(dischargeSubsystem);
        }

        @Override
        public void execute() {
            dischargeSubsystem.setFlyWheelPower(flyWheelPower.get());
            double power = (wantedPos - dischargeSubsystem.getTurretPosition()) * kp;
            dischargeSubsystem.setTurretPower(power);
            dischargeSubsystem.setRampDegree(rampDegree.get());
        }
    }
}
