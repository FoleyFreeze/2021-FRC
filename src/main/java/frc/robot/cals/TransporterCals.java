package frc.robot.cals;

public class TransporterCals extends CalSet {

    public boolean disabled = false;
    //public MotorCal rotateMotor = MotorCal.spark(12).pid(0.2, 0, 0.2, 0).invert().limit(0.6).ramp(0.3).currLim(30).currLimCount(25).currLimTime(1);
    public MotorCal rotateMotor = MotorCal.spark(12).pid(0.2, 0, 0.2, 0).invert().limit(0.10).ramp(0.3).currLim(30).currLimCount(25).currLimTime(1); //MrC
    public MotorCal loadMotor = MotorCal.srx(13).invert(); //this is the gate wheels and the CW motor
    public int CWNotTransport = 5;//3 4 or 5
    public int sensorValue = 5;
    public double hasBallMaxV = 1.1;     //was 2.25 max (MrC)
    public double hasBallMinV = .90;      //was 1.5  min (MrC)
    public int launcherValue = 2;
    public double countsPerIndex = 24/40.0 * 52/36.0 * 64/20.0 * 64/13.0;
    public double allowedIndexError = 0.2 * countsPerIndex;
    public final double TN_LOADSPEED = .95;//0.85;//.5
    public final double TN_STOPSPEED = -0.0;
    public double gateRestTime = 2;
    public double maxGateCurr = 29; //MrC 7.5;
    public double ballSenseDelay = 0.1;//0.9;// MrC 0.1;//.4 //Tried 3 and it never stopped - Mr C
    public double jamRestTime = 2;
    public int maxBallCt;

    public TransporterCals(PneumaticsCals pneumaticsCals){
        if(pneumaticsCals.hardwarePresent){
            maxBallCt = 5;
        } else{
            maxBallCt = 4;
        }

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