package frc.robot.cals;

import frc.robot.cals.MotorCal.MotorType;

public class CannonCals extends CalSet {

    public boolean disabled = false;
    public MotorCal ccMotor = new MotorCal(MotorType.TALON_SRX, 2).limit(1);
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
    public double kPDrive = 0.05;
    public double kDDrive = 0.05;
    public double maxRot = 0.2;
    public double tolerance = 3.0;
    public double initJogDist = 0.0;
    public double initJogAng = 0.0;
    public double distJog = 0.5;
    public double angJog = 1.0;
    public double shootTime = 1.5;
    public double shootCentX = 0.0;
    public double shootCentY = 0.0;
    public double[][] rpm = {{2700.0, 2775.0, 3000.0},
                             {2700.0, 2900.0, 3000.0},
                             {2700.0, 3000.0, 4000.0}, 
                             {2700.0, 3350.0, 5400.0}};
    public double[][] dist = {{  -1.0,  -1.0,  -0.1},
                              { -0.5, 0.0, 35.0},
                              { 30.0, 60.0, 70.0},
                              {60.0, 83.0, 500.0}};
    public double allowedRpmError = 150;
    public double allowedRpmHyst = 150;
    public final double SOL_RESTTIME = 0.2;                                

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