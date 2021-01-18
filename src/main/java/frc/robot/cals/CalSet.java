package frc.robot.cals;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.subsystems.Display;

public class CalSet {

    public static enum BotType {
        COMPETITION, PRACTICE, LASTYEAR
    }
    public static BotType type;

    public static void identifyBot(){
        DigitalInput di = new DigitalInput(9);//TODO: make this a constant id
        DigitalInput lastYearDi = new DigitalInput(8);
        if(!lastYearDi.get()){
            type = BotType.LASTYEAR;
        } else if(di.get()){
            type = BotType.COMPETITION;
        } else {
            type = BotType.PRACTICE;
        }
        di.close();
        lastYearDi.close();

        Display.put("RobotType", type.toString());
        System.out.println("RobotType: " + type);
    } 

    public static boolean isCompBot(){
        return type == BotType.COMPETITION;
    }
    
}