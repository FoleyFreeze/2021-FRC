package frc.robot.cals;

public class PneumaticsCals extends CalSet{
    public final double minPressure = 50;
    public final double maxPressure = 70;
    public final double hystPressure = 20;
    public boolean disabled = false;
    public final double pauseTime = 5.0;

    public final int PNE_PSENSORID = 4;

    public PneumaticsCals(){
        switch(type){
            case COMPETITION:

            break;

            case PRACTICE:
                disabled = true;
            break;

            case LASTYEAR:

            break;
        }
    }
}