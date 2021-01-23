package frc.robot.cals;

public class ClimberCals extends CalSet {

    public boolean disabled = true;
    public int dropFootValue = 4;//3 4 or 5
    public static double upPower = 0.2;
    public static double dnPower = -0.2;
    public static double shiftUpClimb = 0.7;
    public static double shiftDnClimb = -0.7;


    public ClimberCals(){

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