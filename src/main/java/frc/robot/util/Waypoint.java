package frc.robot.util;

import java.io.File;
import java.util.ArrayList;
import java.util.Scanner;

import edu.wpi.first.wpilibj.Filesystem;

public class Waypoint{
    
    public final double x;
    public final double y;
    public final double theta;

    public Waypoint(double x, double y, double theta){
        this.x = x;
        this.y = y;
        this.theta = theta;
    }

    public String toString(){
        return String.format("%.1fin  %.1fin  %.0fdeg",x,y,theta);
    }

    public static Waypoint[] fromFile(String filename){
        try{

            File f = new File(Filesystem.getDeployDirectory(), filename);
            
            Scanner in = new Scanner(f);

            ArrayList<Waypoint> list = new ArrayList<>();
            while(in.hasNextDouble()){
                Waypoint w = new Waypoint(in.nextDouble(), in.nextDouble(), in.nextDouble());
                list.add(w);
            }

            in.close();

            Waypoint[] points = new Waypoint[list.size()];
            for(int i=0;i<list.size();i++){
                points[i] = list.get(i);
            }
            return points;

        } catch (Exception e){
            e.printStackTrace();
        }

        return null;
    }
}