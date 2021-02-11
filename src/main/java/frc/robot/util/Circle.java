package frc.robot.util;

import java.io.File;
import java.util.ArrayList;
import java.util.Scanner;

import edu.wpi.first.wpilibj.Filesystem;

public class Circle {
    public double radius;
    Waypoint center;
    Waypoint end;

    //this is a circle
    public Circle(Waypoint end, Waypoint center){
        //this.radius = radius;
        radius = Math.sqrt((end.x-center.x)*(end.x-center.x) + (end.y-center.y)*(end.y-center.y));
        this.center = center;
        this.end = end;
    }

    //this, despite the name, is a line
    public Circle(Waypoint end){
        //this.radius = radius;
        this.end = end;
        radius = 0;
    }

    public double tangentAngle(double botX, double botY, double botAngle){
        //(botY - center.y)*(botY - center.y)=radius*radius-((botX - center.x)*(botX - center.x));
        double radAngle = Math.atan2(botY-center.y, botX-center.x);
        double delta = Util.angleDiffRad(radAngle+Math.PI/2, botAngle);
        if (delta>Math.PI/2){
            return radAngle - Math.PI/2;
        } else {
            return radAngle + Math.PI/2;
        }

    }

    public double getX(){
        return end.x;
    }

    public double getY(){
        return end.y;
    }

    public double getCentX(){
        return center.x;
    }

    public double getCentY(){
        return center.y;
    }

    public double angleofEnd(){
        Vector endRadius = Vector.fromXY(end.x-center.x, end.y-center.y);
        return endRadius.theta;
    }

    public static ArrayList<Circle> fromFile(String filename){
        try{

            File f = new File(Filesystem.getDeployDirectory(), filename);
            
            Scanner in = new Scanner(f);

            ArrayList<Circle> list = new ArrayList<>();
            while(in.hasNextLine()){
                String line = in.nextLine();
                String[] parts = line.split("[ ,]");
                Circle w;

                if(parts.length == 2){//line segment
                    w = new Circle(new Waypoint(Double.parseDouble(parts[0]), Double.parseDouble(parts[1]), 0));
                } else if(parts.length == 4){//this is a circle
                    Waypoint end = new Waypoint(Double.parseDouble(parts[0]), Double.parseDouble(parts[1]), 0);
                    Waypoint center = new Waypoint(Double.parseDouble(parts[2]), Double.parseDouble(parts[3]), 0);
                    w = new Circle(end, center);
                } else{//uh-oh
                    System.out.println("ERROR: "+ line);
                    w = null;
                }
                
                list.add(w);
            }

            in.close();
            return list;
        } catch (Exception e){
            e.printStackTrace();
        }

        return null;
    }
}
