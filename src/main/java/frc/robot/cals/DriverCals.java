package frc.robot.cals;

public class DriverCals extends CalSet {

    public boolean disabled = false;

    public MotorCal[] driveMotors = {   
        MotorCal.spark(20).ramp(0.3).limit(1.0).coast().currLim(80), 
        MotorCal.spark( 1).ramp(0.3).limit(1.0).coast().currLim(80), 
        MotorCal.spark(14).ramp(0.3).limit(1.0).coast().currLim(80), 
        MotorCal.spark(15).ramp(0.3).limit(1.0).coast().currLim(80)};
    public MotorCal[] turnMotors = {   
        MotorCal.spark( 5).pid(0.2, 0, 0.25, 0).dFilt(0.01).limit(0.50).brake().ramp(0.001), 
        MotorCal.spark( 4).pid(0.2, 0, 0.25, 0).dFilt(0.01).limit(0.50).brake().ramp(0.001), 
        MotorCal.spark(10).pid(0.2, 0, 0.25, 0).dFilt(0.01).limit(0.50).brake().ramp(0.001), 
        MotorCal.spark(11).pid(0.2, 0, 0.25, 0).dFilt(0.01).limit(0.50).brake().ramp(0.001)};
    public int[] turnEncoderIds = {2, 1, 3, 0};
    public double[] xPos = {-10.75, 10.75, -10.75, 10.75};
    public double[] yPos = {12.5, 12.5, -12.5, -12.5};
    public double[] angleOffset;

    public double turnTicksPerRev = 60.0;

    public double driveTicksPerIn = 64.0/18.0 * 18.0/32.0 * 45.0/15.0 / (/*4.0*/Math.PI);
    public double[] wheelDiam = {4, 4, 4, 4};

    public double pausePwrPne = 0.9;//driving over this power will pause compressor

    public double parkOffset = 6000;//0.5;

    public double driveStraightKp = 0.05;//100% after abt 100deg of error
    
    public double trenchRunAngKp = 0.01;
    public double trenchRunDistKp = 0.01;
    public double trenchRunMaxSpd = 1;

    public double autoShootAngKp = 0;
    public double autoBallAngKp = 0;
    public double autoBallDistKp = 0;

    public double autoDriveStrafeKp = 0.1; //full power in 10in
    public double autoDriveAngKp = 0.02; //50deg is full power
    public double autoDriveStrafeRange = 3;
    public double autoDriveAngRange = 5;
    public double autoDriveMaxPwr = 0.15;
    public double autoDriveStartPwr = 0.15;
    public double autoDriveEndPwr = 0.15;
    public double autoDriveStartDist = 36;
    public double autoDriveEndDist = 36;

    public final double DRV_XROBOTCENT = 0.0;
    public final double DRV_YROBOTCENT = 0.0;

    public double gathererDist = 6.75;
    public double climberDist = 8.5;

    public final double DRV_TURNDEADBND = 0.6;

    public final double DRV_GATHERKP = 0.01;

    public DriverCals(){

        switch(type){
            case COMPETITION:
                                        //these are in encoder order (not wheel order) 
                angleOffset = new double[]{2.764, 3.735, 3.547, 0.085};
            break;

            case PRACTICE:
                angleOffset = new double[]{3.749,1.290,0.536,2.169};
            break;

            case LASTYEAR:
                angleOffset = new double[]{0.988,4.673,0.688, 1.757};
                
                driveMotors[0] = MotorCal.spark( 1).ramp(0.3).limit(1.0).coast().invert();
                driveMotors[1] = MotorCal.spark(14).ramp(0.3).limit(1.0).coast().invert();
                driveMotors[2] = MotorCal.spark(20).ramp(0.3).limit(1.0).coast().invert();
                driveMotors[3] = MotorCal.spark(15).ramp(0.3).limit(1.0).coast().invert();

                turnMotors[0] = MotorCal.spark( 5).pid(0.25, 0, 0.2, 0).dFilt(0.01).limit(0.6).brake().ramp(0.001);
                turnMotors[1] = MotorCal.spark(11).pid(0.25, 0, 0.2, 0).dFilt(0.01).limit(0.6).brake().ramp(0.001); 
                turnMotors[2] = MotorCal.spark( 4).pid(0.25, 0, 0.2, 0).dFilt(0.01).limit(0.6).brake().ramp(0.001); 
                turnMotors[3] = MotorCal.spark(10).pid(0.25, 0, 0.2, 0).dFilt(0.01).limit(0.6).brake().ramp(0.001);

                turnEncoderIds[0] = 0;
                turnEncoderIds[1] = 1;
                turnEncoderIds[2] = 2;
                turnEncoderIds[3] = 3;

                driveTicksPerIn = 24.0/10.0 * 2.0/1.0 /(3*Math.PI) ;
                turnTicksPerRev = 60.0/20.0 * 60.0/20.0 * 64.0/40.0 ;
            break;

        }

    }

}