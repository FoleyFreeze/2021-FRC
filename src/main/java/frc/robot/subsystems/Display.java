package frc.robot.subsystems;

import java.util.HashMap;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Display extends SubsystemBase{

    public static HashMap<String, NetworkTableEntry> map = new HashMap<>();
    static NetworkTableEntry error;

    public Display(){
    }

    //TODO: figure out how to add the same data to multiple tabs
    public static void addToTab(ShuffleboardTab tab, String name, double init, int x, int y){
        if(map.containsKey(name)){
            NetworkTableEntry nte = map.get(name);
            //tab.add(name, nte.getValue());
        } else {
            NetworkTableEntry nte = tab.add(name, init).withPosition(x, y).getEntry();
            map.put(name,nte);
        }
    }
    public static void addToTab(ShuffleboardTab tab, String name, String init, int x, int y){
        if(map.containsKey(name)){
            NetworkTableEntry nte = map.get(name);
            //tab.add(name, nte.getValue());
        } else {
            NetworkTableEntry nte = tab.add(name, init).withPosition(x, y).getEntry();
            map.put(name,nte);
        }
    }
    public static void addToTab(ShuffleboardTab tab, String name, boolean init, int x, int y){
        if(map.containsKey(name)){
            NetworkTableEntry nte = map.get(name);
            //tab.add(name, nte.getValue());
        } else {
            NetworkTableEntry nte = tab.add(name, init).withPosition(x, y).getEntry();
            map.put(name,nte);
        }
    }

    public static void init(){
        ShuffleboardTab tab = Shuffleboard.getTab("Comp");
            addToTab(tab, "Selected Auton", "default", 1, 0);
            addToTab(tab, "Pi Alive", false, 2, 0);
            addToTab(tab, "Last Target", "t, d, theta", 3, 0);
            addToTab(tab, "Last Ball", "t, d, theta", 4, 0);
            addToTab(tab, "DT", 0, 5, 0);
            addToTab(tab, "Image DT", 0, 6, 0);
            addToTab(tab, "Robo Pos", "x, y", 7, 0);
            addToTab(tab, "NavX Ang", 0, 0, 1);
            addToTab(tab, "Motors Good", true, 1, 1);
            addToTab(tab, "Ball Number", 1, 2, 1);
            addToTab(tab, "FieldColor", "nope", 3, 1);
            addToTab(tab, "CompressorRun", false, 4, 1);
            addToTab(tab, "Pressure", 0, 5, 1);
            addToTab(tab, "Pit Mode", false, 4, 3);
            addToTab(tab, "Last Error", "N/A", 6, 1);
            addToTab(tab, "RobotType", "Imaginary", 7,1);
            addToTab(tab, "JogUpDn", 0, 8,1);
            addToTab(tab, "JogLR", 0, 8,2);
            addToTab(tab, "RPM", 0, 8, 0);
            addToTab(tab, "Counter", 0, 9,0);
        
        tab = Shuffleboard.getTab("DriveTrain");
            addToTab(tab, "DMotorCurrent 0", 0, 0, 0);
            addToTab(tab, "DMotorCurrent 1", 0, 0, 1);
            addToTab(tab, "DMotorCurrent 2", 0, 0, 2);
            addToTab(tab, "DMotorCurrent 3", 0, 0, 3);
            addToTab(tab, "DMotorTemp 0", 0, 1, 0);
            addToTab(tab, "DMotorTemp 1", 0, 1, 1);
            addToTab(tab, "DMotorTemp 2", 0, 1, 2);
            addToTab(tab, "DMotorTemp 3", 0, 1, 3);
            addToTab(tab, "TMotorCurrent 0", 0, 2, 0);
            addToTab(tab, "TMotorCurrent 1", 0, 2, 1);
            addToTab(tab, "TMotorCurrent 2", 0, 2, 2);
            addToTab(tab, "TMotorCurrent 3", 0, 2, 3);
            addToTab(tab, "TMotorTemp 0", 0, 3, 0);
            addToTab(tab, "TMotorTemp 1", 0, 3, 1);
            addToTab(tab, "TMotorTemp 2", 0, 3, 2);
            addToTab(tab, "TMotorTemp 3", 0, 3, 3);
            addToTab(tab, "Field Relative Pos","x, y", 4,1);
            addToTab(tab, "NavX Ang", 0, 4,0);
            addToTab(tab, "DistSenseInfo Ri", "n/a", 4,2);
            addToTab(tab, "DistSenseInfo Re", "n/a", 4,3);

        tab = Shuffleboard.getTab("Climber");
            addToTab(tab, "Elevator Pos", 0, 0, 0);
            addToTab(tab, "Foot Dropped", false, 1, 0);
            addToTab(tab, "CCMotorCurrent 0", 0, 2, 0);
            addToTab(tab, "CCMotorCurrent 1", 0, 2, 1);
            addToTab(tab, "CC Motor Temp 0", 0, 3, 0);
            addToTab(tab, "CC Motor Temp 1", 0, 3, 1);

        tab = Shuffleboard.getTab("Intake");
            addToTab(tab, "InMotorCurr", 0, 0, 0);    
            addToTab(tab, "Extended", false, 1, 0);
            addToTab(tab, "Ball Spotted", false, 2, 0);
            addToTab(tab, "Ball Info", "dist, ang", 3, 0);

        tab = Shuffleboard.getTab("Cannon");
            addToTab(tab, "RPM", 0, 0, 0);
            addToTab(tab, "Ball Number", 0, 1, 0);
            addToTab(tab, "Hood Pos", "pos", 2, 0);
            addToTab(tab, "Last Target Info", "dist, ang", 0, 3);
            addToTab(tab, "Has Target", false, 3, 0);
            addToTab(tab, "CCMotorCurrent 0", 0, 4, 0);
            addToTab(tab, "CCMotorCurrent 1", 0, 4, 1);
            addToTab(tab, "CC Motor Temp 0", 0, 5, 0);
            addToTab(tab, "CC Motor Temp 1", 0, 5, 1);


        tab = Shuffleboard.getTab("Transporter");
            addToTab(tab, "TCMotorCurrent0", 0, 0, 0);
            addToTab(tab, "TCMotorCurrent1", 0, 0, 1);
            addToTab(tab, "TC Motor Temp 0", 0, 1, 0);
            addToTab(tab, "TC Motor Temp 1", 0, 1, 1);
            addToTab(tab, "Ball Number", 0, 2, 0);
            addToTab(tab, "Ball Position 0", false, 3, 0);
            addToTab(tab, "Ball Position 1", false, 3, 1);
            addToTab(tab, "Ball Position 2", false, 3, 2);
            addToTab(tab, "Ball Position 3", false, 3, 3);
            addToTab(tab, "Ball Position 4", false, 3, 4);
            addToTab(tab, "Current Pos", 0, 4, 0);

        tab = Shuffleboard.getTab("Color Wheel");
            addToTab(tab, "TCMotorCurrent0", 0, 0, 0);
            addToTab(tab, "TCMotorCurrent1", 0, 0, 1);
            addToTab(tab, "TC Motor Temp 0", 0, 0, 2);
            addToTab(tab, "TC Motor Temp 1", 0, 0, 3);
            addToTab(tab, "Color Info", "RGB, Ir, p", 1, 0);
            addToTab(tab, "Field Given Color", "null", 2, 0);
            addToTab(tab, "Detected Color", "null", 3, 0);
    }

    public static void put(String name, double data){
        if(map.containsKey(name)){
            map.get(name).setDouble(data);
        } else {
            error.setString(name);
        }
    }

    public static void put(String name, boolean data){
        if(map.containsKey(name)){
            map.get(name).setBoolean(data);
        } else {
            error.setString(name);
        }
    }

    public static void put(String name, String data){
        if(map.containsKey(name)){
            map.get(name).setString(data);
        } else {
            error.setString(name);
        }
    }
}