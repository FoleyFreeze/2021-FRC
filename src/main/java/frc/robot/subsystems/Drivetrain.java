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
import frc.robot.subsystems.Vision.VisionData;
import frc.robot.util.DistanceSensors;
import frc.robot.util.Util;
import frc.robot.util.Vector;
import frc.robot.util.DistanceSensors.DistData;

import com.kauailabs.navx.frc.AHRS;

public class Drivetrain extends SubsystemBase{
    public class Wheel{
        DriverCals cals;
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
        double initialAng;
        
        //create a wheel object to make assigning motor values easier
        public Wheel(DriverCals cals, int idx){
            this.cals = cals;
            driveMotor = Motor.initMotor(cals.driveMotors[idx]);
            turnMotor = Motor.initMotor(cals.turnMotors[idx]);
            enc = new AnalogInput(cals.turnEncoderIds[idx]);
            location = Vector.fromXY(cals.xPos[idx], cals.yPos[idx]);
            this.idx = idx;
            this.angleOffset = cals.angleOffset[cals.turnEncoderIds[idx]];
            state = new SwerveModuleState();
            prevTime = 0;
            prevPos = 0;
            initialAng = ((enc.getVoltage() - angleOffset)*2*Math.PI/5);
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
            
            //SmartDashboard.putNumber("RawEnc" + idx, encVoltage);
            double currentAngle;
            if(k.analogVTicks){
                double encVoltage = enc.getVoltage();
                currentAngle = ((encVoltage - angleOffset) 
                    *2*Math.PI/5);
            } else{
                currentAngle = turnMotor.getPosition()/k.turnTicksPerRev*2*Math.PI + initialAng;
            }
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
            //SmartDashboard.putNumber("CurrAngle" + idx, Math.toDegrees(currentAngle));
            //SmartDashboard.putNumber("AngleCorr" + idx, Math.toDegrees(angleDiff));
            //SmartDashboard.putString("WheelCmd" + idx, wheelVec.toString());
            

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

        public SwerveModuleState getState(double dt){
            double pos = -driveMotor.getPosition();
            Rotation2d wheelAng; 
            if(cals.analogVTicks){
                wheelAng = new Rotation2d((enc.getVoltage() - angleOffset) * 2*Math.PI / 5);
            } else {
                wheelAng = new Rotation2d(turnMotor.getPosition()/k.turnTicksPerRev*2*Math.PI + initialAng);
            }
            
            double velocity = (pos - prevPos)/dt / k.driveInPerTick * k.wheelDiam[idx];
            
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
    public Vector recentVelocity;

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
            Thread.sleep(800);
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

        prevAng = 0;
        goalAng = 0;
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

    public void drive(Vector strafe, double rot, boolean fieldOrient){
        drive(strafe, rot, 0, 0, fieldOrient);
    }

    public void drive(Vector strafe, double rot, double maxPower){
        drive(strafe, rot, 0, 0, mSubsystem.m_input.fieldOrient(), maxPower);
    }

    public void drive(Vector strafe, double rot, double maxPower, boolean fieldOrient){
        drive(strafe, rot, 0, 0, fieldOrient, maxPower);
    }

        //joystick x, joystick y, joystick rot, center of rotation x and y, field oriented
        //boolean botRelFirst = false;
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
            //botRelFirst = true;
        } /*else if(botRelFirst){
            setStartPosition(0.0, 0.0);
            //botRelFirst = false;
        }*/
        
        //SmartDashboard.putBoolean("Driving Straight", driveStraight);
        
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
        if(resetDriveStraight(rot)) goalAng = prevAng;

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

    private boolean rotGood = false;
    private boolean strafeGood = false;
    private double rotPwr;
    private double maxPwr;
    private Vector strafePwr;

    public void driveRot(double rotPwr, double maxPwr){
        rotGood = true;
        if(strafeGood) drive(strafePwr, rotPwr, Math.min(maxPwr, this.maxPwr));
        this.rotPwr = rotPwr;
        this.maxPwr = maxPwr;
    }

    public void driveStrafe(Vector strafePwr, double maxPwr){
        strafeGood = true;
        SmartDashboard.putString("StrafeCmd",strafePwr.toString());
        if(rotGood) drive(strafePwr, rotPwr, Math.min(maxPwr, this.maxPwr));
        this.strafePwr = strafePwr;
        this.maxPwr = maxPwr;
    }

    public void driveStrafe(Vector strafePwr, double maxPwr, boolean fieldOrient){
        strafeGood = true;
        SmartDashboard.putString("StrafeCmd",strafePwr.toString());
        if(rotGood) drive(strafePwr, rotPwr, Math.min(maxPwr, this.maxPwr), fieldOrient);
        this.strafePwr = strafePwr;
        this.maxPwr = maxPwr;
    }

    public void resetRotStrafe(){
        rotGood = false;
        strafeGood = false;
    }

    private boolean resetDriveStraight(double rot){
        return rot != 0 
            || Math.abs(Util.angleDiff(prevAng, goalAng)) > k.driveStraightMaxDelta;
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

    public double dt;
    double prevTime = Timer.getFPGATimestamp();
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

        //Display.put("DistSenseInfo Re", distSens.getRear().toString());
        //Display.put("DistSenseInfo Ri", distSens.getRight().toString());

        robotAng = -navX.getAngle();
        Rotation2d robotRot2d = new Rotation2d(Math.toRadians(robotAng)/* - Math.PI/2*/);
        Display.put("NavX Ang", robotAng);

        double time = Timer.getFPGATimestamp();
        dt = time - prevTime;
        prevTime = time;
        SwerveModuleState s1 = wheels[0].getState(dt);
        SwerveModuleState s2 = wheels[1].getState(dt);
        SwerveModuleState s3 = wheels[2].getState(dt);
        SwerveModuleState s4 = wheels[3].getState(dt);
        drivePos = driveOdom.update(robotRot2d, s1, s2, s3, s4);
        recentVelocity = getDriveVel(drivePos, dt);

        double x = drivePos.getTranslation().getX();
        double y = drivePos.getTranslation().getY();
        Display.put("Robo Pos", String.format("%.0f, %.0f",x,y));
        Display.put("Motors Good", motorsGood);

        Display.put("Pit Mode", mSubsystem.m_input.pitMode());
        resetRotStrafe();
    }

    public void setStartPosition(double x, double y){
        Rotation2d angle = new Rotation2d(robotAng);
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

    Pose2d prevPos = new Pose2d();

    double avgx;
    double avgy;
    double w = 0.7;
    public Vector getDriveVel(Pose2d pos, double dt){
        double dx = pos.getX() - prevPos.getX();
        double dy = pos.getY() - prevPos.getY();
        prevPos = pos;

        avgx = w*avgx + (1-w)*dx;
        avgy = w*avgy + (1-w)*dy;

        return Vector.fromXY(avgx/dt, avgy/dt);
    }

    public void resetFieldPos(VisionData image){
        //THIS IS DEPENDENT ON STARTING AT 90,125

        double y = image.location[0].r;
        double x = -Math.tan(image.location[0].theta - Math.toRadians(image.robotangle))*y;
        y = (125 + 85) - image.location[0].r;
        x += 90;

        //System.out.printf("%.1f, %.1f\n", x, y);

        setStartPosition(x, y);
    }
}