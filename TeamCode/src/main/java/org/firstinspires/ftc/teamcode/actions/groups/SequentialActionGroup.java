package org.firstinspires.ftc.teamcode.actions.groups;

import com.acmerobotics.roadrunner.SequentialAction;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.actions.ActionBase;

public class SequentialActionGroup {
    public SequentialAction action;
    public SequentialCommandGroup command;
    public SequentialActionGroup(ActionBase... a){
        action = new SequentialAction(a);
        command = new SequentialCommandGroup(a);
    }
}
