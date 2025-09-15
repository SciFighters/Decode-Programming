package org.firstinspires.ftc.teamcode.needle.commands;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.needle.subsystems.ArmAxisSubsystem;
import org.firstinspires.ftc.teamcode.needle.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.needle.subsystems.TelescopicArmSubsystem;

public class Groups {
    public static class GoToBasketCmd extends ParallelCommandGroup {
        public GoToBasketCmd(TelescopicArmSubsystem telescopicArmSubsystem,
                             ArmAxisSubsystem armAxisSubsystem, ClawSubsystem clawSubsystem) {
            addCommands(
                    new TelescopicArmCommands.GoTo(telescopicArmSubsystem, telescopicArmSubsystem.basketPos),
                    new ArmAxisCommands.AxisGoTo(armAxisSubsystem,telescopicArmSubsystem, armAxisSubsystem.basketAngle));
//                new ClawCommands.SetAxisServo(clawSubsystem, clawSubsystem.basketTilt));
        }
    }

    public static class GoHome extends ParallelCommandGroup {
        public GoHome(TelescopicArmSubsystem telescopicArmSubsystem,
                      ArmAxisSubsystem armAxisSubsystem, ClawSubsystem clawSubsystem) {

            addCommands(
                    //not using the go home command because when the angle isnt zero then it closes too much
                    new TelescopicArmCommands.GoTo(telescopicArmSubsystem,30),
                    new ArmAxisCommands.AxisGoTo(armAxisSubsystem,telescopicArmSubsystem, 0));

//                new ClawCommands.SetAxisServo(clawSubsystem, clawSubsystem.intakePos)
        }
    }


}
