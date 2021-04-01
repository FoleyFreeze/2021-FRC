package frc.robot.cals;

public class VisionCals extends CalSet {

    boolean disabled = true;
    public int historySize = 5;
    public double maxImageTimeBall = 0.1;//was 1.2
    public double maxImageTimeTgt = 0.5;
    public boolean coneMode = false;

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