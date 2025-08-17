package org.firstinspires.ftc.teamcode.needle.commands;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.needle.subsystems.ArmAxisSubsystem;
import org.firstinspires.ftc.teamcode.needle.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.needle.subsystems.TelescopicArmSubsystem;

public class Groups {
    public static class GoToBasketCmd extends ParallelCommandGroup {
        public GoToBasketCmd(TelescopicArmSubsystem telescopicArmSubsystem,
                             ArmAxisSubsystem armAxisSubsystem, ClawSubsystem clawSubsystem) {
            addCommands(new TelescopicArmCommands.GoTo(telescopicArmSubsystem, telescopicArmSubsystem.basketPos),
                    new ArmAxisCommands.AxisGoTo(armAxisSubsystem, armAxisSubsystem.basketAngle));
//                new ClawCommands.SetAxisServo(clawSubsystem, clawSubsystem.basketTilt));
        }
    }

    public static class GoHome extends ParallelCommandGroup {
        public GoHome(TelescopicArmSubsystem telescopicArmSubsystem,
                      ArmAxisSubsystem armAxisSubsystem, ClawSubsystem clawSubsystem) {

            addCommands(
                    new SequentialCommandGroup(
                            new ArmAxisCommands.waitForAngle(armAxisSubsystem, 50),
                            new TelescopicArmCommands.GoHome(telescopicArmSubsystem)),
                    new ArmAxisCommands.AxisGoTo(armAxisSubsystem, 0));

//                new ClawCommands.SetAxisServo(clawSubsystem, clawSubsystem.intakePos)
        }
    }

    public static ParallelCommandGroup GoHome(TelescopicArmSubsystem telescopicArmSubsystem,
                                              ArmAxisSubsystem armAxisSubsystem, ClawSubsystem clawSubsystem) {
        return new ParallelCommandGroup(new TelescopicArmCommands.GoHome(telescopicArmSubsystem),
                new ArmAxisCommands.AxisGoTo(armAxisSubsystem, 0));
//                new ClawCommands.SetAxisServo(clawSubsystem, clawSubsystem.intakePos)
    }

}
