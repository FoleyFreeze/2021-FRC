package frc.robot.cals;

public class VisionCals extends CalSet {

    boolean disabled = true;
    public int historySize = 5;
    public double maxImageTime = 0.2;

    public VisionCals(){

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