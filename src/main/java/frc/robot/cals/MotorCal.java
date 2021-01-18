package frc.robot.cals;

public class MotorCal {
    public MotorType type;
    public int id;
    public double minPower;
    public double maxPower;
    public boolean pid = false;
    public double kP;
    public double kI;
    public double kD;
    public double kDFilt;
    public double kF;
    public double rampRate;
    public boolean brake;
    public boolean invert = false;
    public boolean follow = false;
    public int followID;
    public double currentLimit = 50;
    public int overCurrentCountLimit = 50;
    public double overCurrentRestTime = 5;
    public double tempLimit = 60;
    public double overTempRestTime = 30;
    public int overCurrentCountDown = 1;

    public enum MotorType{
        PWM_TALON, SPARK_MAX, TALON_SRX, NULL
    }

    public static MotorCal spark(int id){
        return new MotorCal(MotorType.SPARK_MAX, id);
    }

    public static MotorCal pwm(int id){
        return new MotorCal(MotorType.PWM_TALON, id);
    }

    public static MotorCal srx(int id){
        return new MotorCal(MotorType.TALON_SRX, id);
    }

    MotorCal(MotorType type, int id, double kP, double kI, double kD){

    }
    
    MotorCal(MotorType type, int id, double powerLim){
        this.type = type;
        this.id = id;
        limit(powerLim);
    }

    MotorCal(MotorType type, int id){
        this(type, id, 1);  
    }   

    public MotorCal pid(double p, double i, double d, double f){
        pid = true;
        kP = p;
        kI = i;
        kD = d;
        kF = f;
        return this;
    }

    public MotorCal follow(int id){
        follow = true;
        followID = id;
        return this;
    }

    public MotorCal currLim(double lim){
        currentLimit = lim;
        return this;
    }
    public MotorCal currLimCount(int time){
        overCurrentCountLimit = time;
        return this;
    }
    public MotorCal currLimCntDn(int down){
        overCurrentCountDown = down;
        return this;
    }

    public MotorCal currLimTime(double time){
        overCurrentRestTime = time;
        return this;
    }

    public MotorCal brake(){
        this.brake = true;
        return this;
    }

    public MotorCal invert(){
        this.invert = true;
        return this;
    }

    public MotorCal coast(){
        brake = false;
        return this;
    }

    public MotorCal limit(double min, double max){
        minPower = min;
        if(minPower > 0) minPower = -min;
        maxPower = max;
        if(maxPower < 0) maxPower = -max;
        return this;
    }

    public MotorCal limit(double lim){
        return limit(lim,lim);
    }

    public MotorCal dFilt(double filter){
        kDFilt = filter;
        return this;
    }

    public MotorCal ramp(double rate){
        rampRate = rate;
        return this;
    }
}