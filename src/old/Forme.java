package old;
import java.util.ArrayList;

import com.jme3.math.FastMath;
import com.jme3.math.Matrix4f;
import com.jme3.math.Plane;
import com.jme3.math.Plane.Side;
import com.jme3.math.Quaternion;
import com.jme3.math.Vector3f;

public class Forme {
	
	String name;
	public String toString(){ return name;}
	Forme(String name){ this.name = name;}
	
	boolean landed = true;
	double mass = 1000; // en gramme
	double roundBBRayon=1; //rayon of the boundingbox (en m)
	
	//it's always 0,0,0 !
	//Vector3d gravityCenter = new Vector3d(0,0,0); // en m
	
	//par rapport au centre de gravité
	Vector3d position = new Vector3d(0,0,0); // en m
	Vector3d vitesse = new Vector3d(0,0,0); // en m/s
	Vector3d acceleration = new Vector3d(0,0,0); //en m/ss
	Vector3d lastAccel = new Vector3d(0,0,0); //en m/ss
	
	Quaternion pangulaire = Quaternion.IDENTITY; //en rad
	Vector3d vangulaire = new Vector3d(0,0,0); //en rad/s
	Vector3d aangulaire = new Vector3d(0,0,0); //en rad/ss

	Matrix4f transfoMatrix = new Matrix4f();

	//mesh, en coordonées locale (besoin de passer par transfoMatrix)
	ArrayList<Vector3f> points = new ArrayList<>();
	ArrayList<Triangle> triangles = new ArrayList<>();
	Vector3d calcul1 = new Vector3d();
	Vector3f calculF = new Vector3f();

	
	class Triangle{
		public Triangle(int i, int j, int k) {
			a=i;b=j;c=k;
			float ij = points.get(i).distance(points.get(j));
			float jk = points.get(j).distance(points.get(k));
			float ki = points.get(k).distance(points.get(i));
			bbRound = Math.max(Math.max(ij,jk),ki);
		}

		int a,b,c;
		float bbRound; // roundbounding box
		public String toString(){return a+","+b+","+c;}
	}
	
	public void update(int ms){
		
		//update pos
		System.out.println("posBefore: "+position);
		System.out.println("vitBefore: "+vitesse);
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
		acceleration.set(0,0,0);
		transfoMatrix.setTransform(position.toVec3f(), Vector3f.UNIT_XYZ, pangulaire.toRotationMatrix());
		//apply gravity?
		if(!landed){
//			calculF.set(0, -9.81f, 0);
//			transfoMatrix.invert().multNormal(calculF, calculF);
//			acceleration.addLocal(calcul1.set(calculF));
			acceleration.addLocal(calcul1.set(0, -9.81f, 0));
			//System.out.println("not landed");
		}
//		System.out.println("vitesse:"+vitesse);
	}
	//apply a force in g*m/s2
	public void applyForce(Vector3d force) {
		System.out.println("force = "+force);
		//transform
		force.toVec3fLocal(calculF).divideLocal((float) mass);
//		transfoMatrix.invert().multNormal(calculF, calculF);
//		System.out.println("force transformÃ©e = "+calculF);
		//add to accel
//		System.out.println("force transformÃ©e = "+calcul1.set(calculF));
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
		if(!checkAlmostCollisionWithLand(f2)) return false;

		System.out.println("object1 trsf: "+transfoMatrix);
		System.out.println("object2 trsf: "+f2.transfoMatrix);
		return checkExactCollision(f2, this.vitesse.toVec3f()) || f2.checkExactCollision(this, vitesse.toVec3f().negate());
		//return checkAlmostCollisionWithLand(f2);
	}

	public boolean checkExactCollision(Forme f2, Vector3f dir){
		System.out.println("checkExactCollision for "+this+" to "+f2);
		//check by poly
		Vector3f tempA = new Vector3f();
		Vector3f tempB = new Vector3f();
		Vector3f tempC = new Vector3f();
		Vector3f tempP = new Vector3f();
		Vector3f tempPInPlan = new Vector3f();

		Plane plan = new Plane();
		//get all poly 
		for(Triangle tri : triangles){
			//create the plane for this triangle
			plan.setPlanePoints(transfoMatrix.mult(points.get(tri.a), tempA), 
					transfoMatrix.mult(points.get(tri.b), tempB),
					transfoMatrix.mult(points.get(tri.c), tempC));
			System.out.println("testing tri "+tri);
			//check if it's on the good direction
			if(plan.getNormal().dot(dir) > 0){
				//transform it in the f2 repere
				//check all closes points in the other forme
				for(Vector3f p : f2.points){
					f2.transfoMatrix.mult(p, tempP);
					System.out.println("testing with p "+p+" => "+plan.whichSide(f2.transfoMatrix.mult(p, tempP))+" ("+f2.transfoMatrix.mult(p, tempP)+")");
					if(plan.whichSide(tempP) != Side.Positive){
						//check if inside triangle
						plan.getClosestPoint(tempP, tempPInPlan);
						//'near' the triangle?
						System.out.println("near? "+tempA.distance(tempPInPlan)+" =?= "+tri.bbRound);
						if(tempA.distance(tempPInPlan) < tri.bbRound){
							//ok, do the math
							//move inside the new repere
							
							// Compute vectors        
							Vector3f v0 = tempC.subtract(tempA);
							Vector3f v1 = tempB.subtract(tempA); 
							Vector3f v2 = tempPInPlan.subtract(tempA);

							// Compute dot products
							float dot00 = v0.dot(v0);
							float dot01 = v0.dot(v1);
							float dot02 = v0.dot(v2);
							float dot11 = v1.dot(v1);
							float dot12 = v1.dot(v2);

							// Compute barycentric coordinates
							float invDenom = 1 / (dot00 * dot11 - dot01 * dot01);
							float u = (dot11 * dot02 - dot01 * dot12) * invDenom;
							float v = (dot00 * dot12 - dot01 * dot02) * invDenom;

							// Check if point is in triangle
							System.out.println("inside? u="+u+", v="+v+", u+v="+(u+v));
							boolean inside = (u >= 0) && (v >= 0) && (u + v < 1);
							if(inside){
								return true;
							}
						}
					}
				}
			}
			
		}
		
		return false;
	}
	
	public boolean checkAlmostCollisionWithLand(Forme f2){

		Vector3d directionObj1 = new Vector3d(vitesse);
		if(directionObj1.lengthSquared() == 0){
//			calcul1.set(0,-1,0);
//			directionObj1.set(transfoMatrix.invert().multNormal(calcul1.toVec3fLocal(calculF), calculF));
			directionObj1.set(0, -1, 0);
		}
		//Vector3d directionWorld = new Vector3d().set(transfoMatrix.multNormal(directionObj1.toVec3fLocal(calculF), calculF));
		//Vector3d directionObj2 = new Vector3d().set(f2.transfoMatrix.invert().multNormal(directionWorld.toVec3fLocal(calculF), calculF));
		Vector3d directionObj2 = new Vector3d(directionObj1).multLocal(-1);
		
		Vector3d point1 = getMostFarAwayPoint(directionObj1, new Vector3d());
		Vector3d point2 = f2.getMostFarAwayPoint(directionObj2, new Vector3d());
		System.out.println("p1: "+point1);
		System.out.println("p2: "+point2);
		
		//check plane
		Plane plan = new Plane();
		plan.setOriginNormal(point1.toVec3f(), directionObj1.toVec3fLocal(calculF));

		System.out.println("plan: "+plan);
		Side s = plan.whichSide(point2.toVec3f());
		System.out.println("side: "+s);
		
		return s != Side.Positive;
	}
	
	public Vector3d getMostFarAwayPoint(Vector3d direction, Vector3d returnVal){

		//get our more far away point in moving direction

		transfoMatrix.invert().multNormal(direction.toVec3fLocal(calculF), calculF).normalizeLocal();
		//System.out.println("direction: "+calcul1);
		Vector3f farAway = null;
		double score = 0;
		for(Vector3f p : points){
			double newScore = calculF.dot(p);
			//System.out.println("check for point "+p+" : "+newScore);
			if(newScore > score){
				farAway = p;
				score = newScore;
			}
		}
		if(farAway == null){
			System.err.println("error, a forme without point in a direction!");
		}
		
		//TODO: passer cela en doublePrecision.
		//now transform it in world position
		System.out.println("use point "+farAway);
		transfoMatrix.mult(farAway, calculF);
		System.out.println("in world space: "+calculF);
		
		return returnVal.set(calculF);
	}
	
	//set tri to not face the center
	public void doNotFaceCenter() {
		Vector3f tempA = new Vector3f();
		Vector3f tempB = new Vector3f();
		Vector3f tempC = new Vector3f();
		Vector3f tempP = new Vector3f(0,0,0);

		Plane plan = new Plane();
		for(Triangle tri : triangles){
			plan.setPlanePoints(transfoMatrix.mult(points.get(tri.a), tempA), 
					transfoMatrix.mult(points.get(tri.b), tempB),
					transfoMatrix.mult(points.get(tri.c), tempC));	//create plane
			if(plan.whichSide(tempP) == Side.Positive){
				System.out.println("Forme "+this+" : wrong side for triangle "+tri.a+tri.b+tri.c);
				int temp = tri.b;
				tri.b = tri.a;
				tri.a = temp;
			}
		}
		
	}
	
}
