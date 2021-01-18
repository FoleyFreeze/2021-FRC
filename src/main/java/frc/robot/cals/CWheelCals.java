package frc.robot.cals;

import com.revrobotics.ColorMatch;

import edu.wpi.first.wpilibj.util.Color;

public class CWheelCals extends CalSet {

    public boolean disabled = true;
    public double rotSpeed = 0.75;
    public double colSpeed = 0.5;
    public double cwInitSpeed = 0.2;
    public double stopBuffer = 0.5;
    public final Color Red = ColorMatch.makeColor(0.561, 0.232, 0.114);
    public final Color Blue = ColorMatch.makeColor(0.143, 0.427, 0.429);
    public final Color Green = ColorMatch.makeColor(0.197, 0.561, 0.240);
    public final Color Yellow = ColorMatch.makeColor(0.361, 0.524, 0.113);

    public CWheelCals(){

        switch(type){
            case COMPETITION:

            break;

            case PRACTICE:

            break;

            case LASTYEAR:

            break;
        }
    }
}