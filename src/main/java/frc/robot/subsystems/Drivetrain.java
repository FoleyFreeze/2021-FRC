package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.cals.DriverCals;
import frc.robot.motors.Motor;
import frc.robot.util.DistanceSensors;
import frc.robot.util.Util;
import frc.robot.util.Vector;
import frc.robot.util.DistanceSensors.DistData;

import com.kauailabs.navx.frc.AHRS;

public class Drivetrain extends SubsystemBase{
    public class Wheel{
        int idx;
        Motor driveMotor;
        Motor turnMotor;
        AnalogInput enc;
        Vector location;
        Vector rotVec;
        Vector wheelVec;
        double angleOffset;
        double prevTime;
        double prevPos;
        SwerveModuleState state;
        
        //create a wheel object to make assigning motor values easier
        public Wheel(DriverCals cals, int idx){
            driveMotor = Motor.initMotor(cals.driveMotors[idx]);
            turnMotor = Motor.initMotor(cals.turnMotors[idx]);
            enc = new AnalogInput(cals.turnEncoderIds[idx]);
            location = Vector.fromXY(cals.xPos[idx], cals.yPos[idx]);
            this.idx = idx;
            this.angleOffset = cals.angleOffset[cals.turnEncoderIds[idx]];
            state = new SwerveModuleState();
            prevTime = 0;
            prevPos = 0;
        }

        public double calcRotVec(double centX, double centY){
            Vector v = Vector.fromXY(centX, centY);
            v.inverse().add(location);
            v.theta -= Math.PI/2;
            rotVec = v;
            //SmartDashboard.putString("Location" + idx, 
            //    location.toString());
            //SmartDashboard.putString("InitRotate" + idx, v.toString());
            return rotVec.r;
        }

        public void drive(boolean parkMode){
            double encVoltage = enc.getVoltage();
            //SmartDashboard.putNumber("RawEnc" + idx, encVoltage);
            double currentAngle = ((encVoltage - angleOffset) 
                *2*Math.PI/5);
            double angleDiff = Util.angleDiffRad(wheelVec.theta, currentAngle);
            //SmartDashboard.putNumber("AngleRaw" + idx, 
            //    Math.toDegrees(angleDiff));

            //angle diff is now between -PI and PI
            //angleDiff = ((angleDiff+Math.PI*2) % (2*Math.PI)) - Math.PI;
            //angleDiff = Math.IEEEremainder(angleDiff, 2*Math.PI) - Math.PI;
            angleDiff = Util.angleDiffRad(angleDiff, Math.PI);

            //SmartDashboard.putNumber("AngleDiff" + idx, Math.toDegrees(angleDiff));

            if(Math.abs(angleDiff) > Math.PI/2){
                wheelVec.r *= -1;
                if(angleDiff < 0) {
                    angleDiff += Math.PI;
                    wheelVec.theta += Math.PI;
                } else if(angleDiff > 0) {
                    angleDiff -= Math.PI;
                    wheelVec.theta -= Math.PI;
                }
            }
            SmartDashboard.putNumber("CurrAngle" + idx, Math.toDegrees(currentAngle));
            //SmartDashboard.putNumber("AngleCorr" + idx, Math.toDegrees(angleDiff));
            SmartDashboard.putString("WheelCmd" + idx, wheelVec.toString());
            

            double deltaTicks = angleDiff / (2*Math.PI) * k.turnTicksPerRev;
            double targetTicks = turnMotor.getPosition();
            if((Math.abs(wheelVec.r) > 0.05 /*|| parkMode*/) && Math.abs(deltaTicks) > k.DRV_TURNDEADBND){ //don't turn unless we actually want to move
                targetTicks += deltaTicks;
            } 
            //SmartDashboard.putNumber("TargetTicks" + idx, targetTicks);
            //SmartDashboard.putNumber("DeltaTicks" + idx, deltaTicks);
            //SmartDashboard.putNumber("DrivePwr"+idx,wheelVec.r);
            driveMotor.setPower(wheelVec.r);
            turnMotor.setPosition(targetTicks);
        }

        public SwerveModuleState getState(){
            double time = Timer.getFPGATimestamp();
            double pos = -driveMotor.getPosition();
            
            double velocity = (pos - prevPos)/(time - prevTime) / k.driveInPerTick * k.wheelDiam[idx];
            Rotation2d wheelAng = new Rotation2d((enc.getVoltage() - angleOffset) * 2*Math.PI / 5);
            prevTime = time;
            prevPos = pos;
            
            state.speedMetersPerSecond = velocity;
            state.angle = wheelAng;
            return state;
        }
    }

    Wheel[] wheels;
    public DriverCals k;
    public AHRS navX;
    private RobotContainer mSubsystem;

    public Drivetrain(DriverCals cals, RobotContainer subsystem){
        k = cals;
        if(cals.disabled) return;
        mSubsystem = subsystem;
        
        int size = k.driveMotors.length;
        wheels = new Wheel[size];
        for(int i=0; i<size; i++){
            wheels[i] = new Wheel(k, i);
        }

        navX = new AHRS(Port.kMXP);
        navX.calibrate();
        try{
            Thread.sleep(200);
        } catch(Exception e){
        }
        navX.zeroYaw();

        fLWheelLoc = new Translation2d(k.xPos[0], k.yPos[0]);
        fRWheelLoc = new Translation2d(k.xPos[1], k.yPos[1]);
        rLWheelLoc = new Translation2d(k.xPos[2], k.yPos[2]);
        rRWheelLoc = new Translation2d(k.xPos[3], k.yPos[3]);

        driveKinematics = new SwerveDriveKinematics(fLWheelLoc, fRWheelLoc, rLWheelLoc, rRWheelLoc);
        driveOdom = new SwerveDriveOdometry(driveKinematics, new Rotation2d());
        distSens = new DistanceSensors();
    }

    public double parkTime = 9000.0;
    public boolean parkMode = false;
    boolean motorsGood = true;
    double currentTime = 0.0;
    public double prevRot;
    public boolean driveStraight = false;
    public double prevAng = 0.0;
    public double goalAng = 0.0;
    public double robotAng = 0;
    public Translation2d fRWheelLoc;
    public Translation2d fLWheelLoc;
    public Translation2d rRWheelLoc;
    public Translation2d rLWheelLoc;
    public SwerveDriveKinematics driveKinematics;
    public SwerveDriveOdometry driveOdom;
    public Pose2d drivePos;
    public DistanceSensors distSens;

    public void drive(Vector strafe, double rot, double centX, double centY, boolean fieldOrient){
        drive(strafe, rot, centX, centY, fieldOrient, 1);
    }
    
    public void drive(Vector strafe, double rot){
        drive(strafe, rot, 0, 0, mSubsystem.m_input.fieldOrient());
    }

        //joystick x, joystick y, joystick rot, center of rotation x and y, field oriented
    public void drive(Vector strafe, double rot, double centX, double centY, boolean fieldOrient, double maxPower){
        if(k.disabled) return;
        currentTime = Timer.getFPGATimestamp();
        //SmartDashboard.putString("Strafe", String.format("%.2f, %.0f", 
        //    strafe.r, Math.toDegrees(strafe.theta)));
        //SmartDashboard.putNumber("prevAngle", prevAng);
        //SmartDashboard.putNumber("RobotAngle", navX.getAngle());
        if(fieldOrient){
            strafe.theta -= Math.toRadians(-navX.getAngle() % 360) /*- Math.PI/2*/;
            //while(strafe.theta > 180) strafe.theta -= 360;
            //while(strafe.theta < -180) strafe.theta += 360;
        }
        
        //SmartDashboard.putBoolean("Driving Straigth", driveStraight);
        
        /*
        if(0 == rot && 0 == strafe.r){
            if(parkTime <= currentTime){
                parkMode = true;
            }
            else parkMode = false;
        }else{
            parkTime = currentTime + k.parkOffset;
            parkMode = false;
        }
        */
        parkMode = false;

        
        driveStraight = (rot == 0 && strafe.r != 0 && mSubsystem.m_input.driveStraight());
        if(rot != 0) goalAng = prevAng;

        if(driveStraight){
            double error = Util.angleDiff(goalAng, -navX.getAngle());
            //SmartDashboard.putNumber("Straight Error", error);
            rot = k.driveStraightKp * error;
        }

        
        //SmartDashboard.putNumber("Goal Angle", goalAng);

        //SmartDashboard.putNumber("Rot", rot);
        //SmartDashboard.putNumber("parkTime", parkTime);
        //SmartDashboard.putBoolean("parkMode", parkMode);
        
        double maxMag = 0;
        for(Wheel w : wheels){//calculate rotation
            double mag = w.calcRotVec(centX, centY);
            maxMag = Math.max(maxMag, Math.abs(mag));
        }
        //SmartDashboard.putNumber("MaxMag",maxMag);


        Wheel maxOut = wheels[0];
        for(Wheel w : wheels){//normalize the rotation
            w.rotVec.r /= maxMag;
            w.rotVec.r *= rot;
            w.wheelVec = Vector.add(w.rotVec, strafe); 
            if(Math.abs(w.wheelVec.r) > Math.abs(maxOut.wheelVec.r)){
                maxOut = w;
            }
            //SmartDashboard.putString("NormRotate" + w.idx, 
            //    w.rotVec.toString());
            //SmartDashboard.putString("RawWheelCmd" + w.idx, 
            //    w.wheelVec.toString());

            if(parkMode){
                w.wheelVec.theta = w.location.theta;
            }

        }
        
        //double maxPower = 1;
        if(mSubsystem.m_input.pitMode()){
            maxPower = Math.min(0.2, maxPower);
        }

        if(Math.abs(maxOut.wheelVec.r) > maxPower){
            double reducRatio = maxPower/Math.abs(maxOut.wheelVec.r);

            strafe.r *= reducRatio;

            for(Wheel w: wheels){
                w.rotVec.r *= reducRatio;
                w.wheelVec= Vector.add(strafe, w.rotVec);
                //w.wheelVec.r *= reducRatio;
            }
        }

        

        for(Wheel w : wheels){
            //SmartDashboard.putString("FinalWheel" + w.idx, 
            //    w.wheelVec.toString());
            w.drive(parkMode);
            //SmartDashboard.putNumber("DrivePower "+w.idx, strafe.r);
            //SmartDashboard.putNumber("Turn Pwr" + w.idx, w.rotVec.r);
        }

        prevAng = -navX.getAngle();
    }

    public double[] getDist(){
        double[] dists = new double[4];
        for(int i = 0 ; i < wheels.length ; i++){
            dists[i] = wheels[i].driveMotor.getPosition() / k.driveInPerTick * k.wheelDiam[i];
        }
        return dists;
    }

    public DistData getRearDist(){
        return distSens.getRear();
    }

    public DistData getRightDist(){
        return distSens.getRight();
    }

    public void periodic(){
        if(k.disabled) return;
        
        for(Wheel w: wheels){
            if(w.idx == 0) motorsGood = true;
            Display.put("DMotorCurrent " + w.idx, wheels[w.idx].driveMotor.getCurrent());
            Display.put("TMotorCurrent " + w.idx, wheels[w.idx].turnMotor.getCurrent());
            Display.put("DMotorTemp " + w.idx, wheels[w.idx].driveMotor.getTemp());
            Display.put("TMotorTemp " + w.idx, wheels[w.idx].turnMotor.getTemp());
            if(wheels[w.idx].driveMotor.getTemp() >= 70) motorsGood = false;
            //SmartDashboard.putNumber("Turn Pwr "+w.idx, wheels[w.idx].turnMotor.getSpeed());
            //SmartDashboard.putNumber("Turn Curr "+w.idx, wheels[w.idx].turnMotor.getCurrent());
        }

        Display.put("DistSenseInfo Re", distSens.getRear().toString());
        Display.put("DistSenseInfo Ri", distSens.getRight().toString());

        robotAng = -navX.getAngle();
        Rotation2d robotRot2d = new Rotation2d(Math.toRadians(robotAng)/* - Math.PI/2*/);
        Display.put("NavX Ang", robotAng);

        drivePos = driveOdom.update(robotRot2d, wheels[0].getState(), wheels[1].getState(), 
            wheels[2].getState(), wheels[3].getState());

        double x = drivePos.getTranslation().getX();
        double y = drivePos.getTranslation().getY();
        Display.put("Robo Pos", String.format("%.0f, %.0f",x,y));
        Display.put("Motors Good", motorsGood);

        Display.put("Pit Mode", mSubsystem.m_input.pitMode());
    }

    public void setStartPosition(double x, double y){
        Rotation2d angle = new Rotation2d();
        driveOdom.resetPosition(new Pose2d(x,y,angle), angle);
    }

    public void zeroAll(){
        navX.zeroYaw();
        driveOdom.resetPosition(new Pose2d(), new Rotation2d(/*-Math.PI/2*/));
    }

    public boolean getBrake(){
        return wheels[0].driveMotor.getBrake();
    }

    public void setBrake(boolean brake){
        for(Wheel w: wheels){
            w.driveMotor.setBrake(brake);
        }
    }
}