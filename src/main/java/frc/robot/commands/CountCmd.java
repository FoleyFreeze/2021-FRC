package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Display;

public class CountCmd extends CommandBase {
    
    RobotContainer bot;
    int count = 0;

    public CountCmd(RobotContainer bot){
        this.bot = bot;
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }

    @Override
    public void initialize(){
        count++;
        Display.put("Counter",count);
    }

}