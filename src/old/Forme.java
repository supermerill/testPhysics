package old;
import java.util.ArrayList;

import com.jme3.math.Matrix4f;
import com.jme3.math.Plane;
import com.jme3.math.Plane.Side;
import com.jme3.math.Quaternion;
import com.jme3.math.Vector3f;

public class Forme {
	
	boolean landed = true;
	double mass = 1000; // en gramme
	
	//it's always 0,0,0 !
	//Vector3d gravityCenter = new Vector3d(0,0,0); // en m
	
	
	Vector3d position = new Vector3d(0,0,0); // en m
	Vector3d vitesse = new Vector3d(0,0,0); // en m/s, in local trsf
	Vector3d acceleration = new Vector3d(0,0,0); //en m/ss, in local trsf
	Vector3d lastAccel = new Vector3d(0,0,0); //en m/ss, in local trsf

	Matrix4f transfoMatrix = new Matrix4f();
	
	Quaternion pangulaire = Quaternion.IDENTITY; //en rad
	Vector3d vangulaire = new Vector3d(0,0,0); //en rad/s, in local trsf
	Vector3d aangulaire = new Vector3d(0,0,0); //en rad/ss, in local trsf
	
	
	ArrayList<Vector3d> points = new ArrayList<>();
	ArrayList<Triangle> triangles = new ArrayList<>();
	Vector3d calcul1 = new Vector3d();
	Vector3f calculF = new Vector3f();

	
	static class Triangle{
		public Triangle(int i, int j, int k) {
			a=i;b=j;c=k;
		}

		int a,b,c;
	}
	
	public void update(int ms){
		
		//update pos
		calcul1.set(vitesse).multLocal(ms*0.001d);
		position.addLocal(calcul1);
		calcul1.set(lastAccel).multLocal(ms*ms*0.000001d);
		position.addLocal(calcul1);
//		position.x += (float)( (((double)vitesse.x)*ms)/1000 + (((((double)acceleration.x)*ms)/1000)*ms)/1000 );
//		position.y += (float)( (((double)vitesse.y)*ms)/1000 + (((((double)acceleration.y)*ms)/1000)*ms)/1000 );
//		position.z += (float)( (((double)vitesse.z)*ms)/1000 + (((((double)acceleration.z)*ms)/1000)*ms)/1000 );
		
		//update vitesse
		calcul1.set(lastAccel).multLocal(ms*0.0005d);
		vitesse.addLocal(calcul1);
		calcul1.set(acceleration).multLocal(ms*0.0005d);
		vitesse.addLocal(calcul1);
//		vitesse.x += (float)( (((double)lastAccel.x)*ms)/2000 + (((double)acceleration.x)*ms)/2000 );
//		vitesse.y += (float)( (((double)lastAccel.y)*ms)/2000 + (((double)acceleration.y)*ms)/2000 );
//		vitesse.z += (float)( (((double)lastAccel.z)*ms)/2000 + (((double)acceleration.z)*ms)/2000 );
		
		Vector3d temp = lastAccel;
		lastAccel = acceleration;
		acceleration = temp;
		transfoMatrix.setTransform(position.toVec3f(), Vector3f.UNIT_XYZ, pangulaire.toRotationMatrix());
		//apply gravity?
		if(!landed){
			calculF.set(0, -9.81f, 0);
			transfoMatrix.invert().multNormal(calculF, calculF);
			acceleration.addLocal(calcul1.set(calculF));
		}
//		System.out.println("vitesse:"+vitesse);
	}
	//apply a force in g*m/s2
	public void applyForce(Vector3d force) {
//		System.out.println("force = "+gravity);
		//transform
		force.toVec3fLocal(calculF).divideLocal((float) mass);
		transfoMatrix.invert().multNormal(calculF, calculF);
//		System.out.println("force transformée = "+calculF);
		//add to accel
//		System.out.println("force transformée = "+calcul1.set(calculF));
		acceleration.addLocal(calcul1.set(calculF));
//		System.out.println("now accel = "+acceleration);
	}
	
	public boolean checkCollision(Forme f2){
		if(f2.landed){
			if(!landed) return checkCollisionWithLand(f2);
		}else{
			if(landed){
				return f2.checkCollisionWithLand(this);
			}else{
				return checkCollisionWithFreeFlight(f2);
			}
		}
		return false;
	}
	
	//TODO
	public boolean checkCollisionWithFreeFlight(Forme f2){
		return false;
	}
	
	public boolean checkCollisionWithLand(Forme f2){

		Vector3d directionObj1 = new Vector3d(vitesse);
		if(directionObj1.lengthSquared() == 0){
			calcul1.set(0,-1,0);
			directionObj1.set(transfoMatrix.invert().multNormal(calcul1.toVec3fLocal(calculF), calculF));
		}
		Vector3d directionWorld = new Vector3d().set(transfoMatrix.multNormal(directionObj1.toVec3fLocal(calculF), calculF));
		Vector3d directionObj2 = new Vector3d().set(f2.transfoMatrix.invert().multNormal(directionWorld.toVec3fLocal(calculF), calculF));
		
		Vector3d point1 = getMostFarAwayPoint(directionObj1, new Vector3d());
		Vector3d point2 = f2.getMostFarAwayPoint(directionObj2, new Vector3d());
		System.out.println("p1: "+point1);
		System.out.println("p2: "+point1);
		
		//check plane
		Plane plan = new Plane();
		plan.setOriginNormal(directionWorld.toVec3fLocal(calculF), point1.toVec3f());
		
		Side s = plan.whichSide(point2.toVec3f());
		System.out.println("side: "+s);
		
		return s != Side.Positive;
	}
	
	public Vector3d getMostFarAwayPoint(Vector3d direction, Vector3d returnVal){

		//get our more far away point in moving direction
		calcul1.set(direction).normalizeLocal();
		System.out.println("direction in local coord: "+direction);
		Vector3d farAway = null;
		double score = 0;
		for(Vector3d p : points){
			double newScore = calcul1.dot(p);
			System.out.println("check for point "+p+" : "+newScore);
			if(newScore > score){
				farAway = p;
			}
		}
		if(farAway == null){
			System.err.println("error, a forme without point in a direction!");
		}
		
		//TODO: passer cela en doublePrecision.
		//now transform it in world position
		farAway.toVec3fLocal(calculF);
		transfoMatrix.mult(calculF, calculF);
		
		return returnVal.set(calculF);
	}
	
}
