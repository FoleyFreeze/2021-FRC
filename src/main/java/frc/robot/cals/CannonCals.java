package frc.robot.cals;

import frc.robot.cals.MotorCal.MotorType;

public class CannonCals extends CalSet {

    public boolean disabled = false;
    //public MotorCal ccMotor = new MotorCal(MotorType.TALON_SRX, 2).limit(1);
    public MotorCal ccMotor = new MotorCal(MotorType.TALON_SRX, 2).limit(1).pid(0.1, 0.0002, 0, 0.05).iLim(409600);
    public MotorCal ccMotor2 = new MotorCal(MotorType.TALON_SRX, 3).invert().follow(2);
    public double falconRpmPerPower = 5400;
    public int hoodSolValue = 1;
    public int stopSolValue = 0;
    public int camLightsSol = 3; //3 4 or 5
    public int ShootVClimbValue = 6;
    public double layupDist = 0.0;
    public double trenchDist = 208.75;
    public double autonDist = 83;
    public double manualPower = 0.5;
    public double kPDrive = -0.013;
    public double kDDrive = -0.0015;
    public double kIDrive = -0.035;
    public double maxI = Math.abs(0.1 / kIDrive);
    public double iZone = 10;
    public double maxRot = 0.4;
    public double tolerance = 1.0;//in degrees
    public double initShootSpeed = 4000;
    public double initJogDist = 0.0;
    public double initJogAng = 0.0;
    public double distJog = 0.5;
    public double distJogShift = .25;
    public double angJog = 0.5;
    public double shootTime = 1.5;//TODO figure out how short this can be
    public double shootCentX = 0.0;
    public double shootCentY = 0.0;
    public double[][] pnuRpm = {{2700.0, 2775.0, 3000.0},
                             {2700.0, 2900.0, 3000.0},
                             {2700.0, 3000.0, 4000.0}, 
                             {2700.0, 3350.0, 5400.0}};
    public double[][] pnuDist = {{  -1.0,  -1.0,  -0.1},
                              { -0.5, 0.0, 35.0},
                              { 30.0, 60.0, 70.0},
                              {60.0, 83.0, 500.0}};
    public double allowedRpmError = 25;
    public double allowedRpmHyst = 25;
    public final double SOL_RESTTIME = 0.2; 
    public boolean pneumaticHood = false;
    public MotorCal hoodMotor = new MotorCal(MotorType.SPARK_MAX, 7).pid(0.1, 0.0001, 0.0, 0).iLim(2).limit(0.3).resetEnc(false).invert();//.brake();
    public double hoodTicksPerInch = 4 * 30/8.0; //30 revs in 8 in * gear ratio of 4:1
    public double[] screwDist = {0, 22, 44, 96, 118, 156, 214};
    public double[] screwRpm = {2900, 3050, 3200, 3775, 3875, 4100, 4900};
    public double[] screwHoodHeight = {0, -0.2, 0.5, 2, 2.182, 2.5, 3.5};
    double x = 0.0;
    public double[] flightTime = {0.4+x, 0.04+x, 0.45+x, 0.5+x, 0.6+x, 0.7+x, 0.8+x};
    public double maxScrewHeight = 5;
    public double minScrewHeight = 0;
    public boolean enableVelOffset = false;
    public boolean enableLatOffset = false;

    public CannonCals(){
        
        switch(type){
            case COMPETITION:

            break;

            case PRACTICE:
                disabled = true;
                /*
                ccMotor = new MotorCal(MotorType.TALON_SRX, 2).limit(-0.55, 0.55);
                ccMotor2 = new MotorCal(MotorType.TALON_SRX, 3).invert().limit(-0.55, 0.55);

                hoodSolValue = 2;
                stopSolValue = 0;
                camLightsSol = -1;
                ShootVClimbValue = -1;
                */
            break;

            case LASTYEAR:

            break;
        }
    }
}