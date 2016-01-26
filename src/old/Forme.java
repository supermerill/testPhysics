package old;
import java.util.ArrayList;
import java.util.HashMap;

import joint.JointPonctuel;

import collision.CollisionPrediction;
import collision.CollisionUpdater;

import com.jme3.math.FastMath;
import com.jme3.math.Matrix4f;
import com.jme3.math.Plane;
import com.jme3.math.Plane.Side;
import com.jme3.math.Quaternion;
import com.jme3.math.Vector3f;

public class Forme {
	
	public String name;
	public String toString(){ return name;}
	Forme(String name){ this.name = name;}
	
	///---------- phys/colision
	public boolean physicUpdate = false;
	public HashMap<Forme, CollisionPrediction> predictions = new HashMap<>(2);
	public HashMap<Forme, JointPonctuel> joint = new HashMap<>(2); //TODO
	public ArrayList<CollisionPrediction> collideAt = new ArrayList<>(1); //TODO
	//-------------
	
	
	public boolean landed = true;
	public double mass = 1000; // en gramme
	public double roundBBRayon=1; //rayon of the boundingbox (en m)
	
	//it's always 0,0,0 !
	//Vector3d gravityCenter = new Vector3d(0,0,0); // en m
	
	//par rapport au centre de gravite
	public Vector3d position = new Vector3d(0,0,0); // en m
	public Vector3f vitesse = new Vector3f(0,0,0); // en m/ms
	public Vector3f acceleration = new Vector3f(0,0,0); //en m/ms*ms
	public Vector3f lastAccel = new Vector3f(0,0,0); //en m/ms*ms
	
	public Quaternion pangulaire = new Quaternion(Quaternion.IDENTITY); //en rad
	public Vector3f vangulaire = new Vector3f(0,0,0); //en rad/ms
	public Vector3f aangulaire = new Vector3f(0,0,0); //en rad/ms*ms
	public Vector3f lastAangulaire = new Vector3f(0,0,0); //en rad/ms*ms

	public Matrix4f transfoMatrix = new Matrix4f();
	
	public ArrayList<Vector3f> forces = new ArrayList<>();
	

	//mesh, en coordon�es locale (besoin de passer par transfoMatrix)
	public ArrayList<Vector3f> points = new ArrayList<>();
	public ArrayList<Triangle> triangles = new ArrayList<>();
	Vector3d calcul1 = new Vector3d();
	public Vector3f calculF = new Vector3f();

	
	public class Triangle{
		public Triangle(int i, int j, int k) {
			a=i;b=j;c=k;
			float ij = points.get(i).distance(points.get(j));
			float jk = points.get(j).distance(points.get(k));
			float ki = points.get(k).distance(points.get(i));
			bbRound = Math.max(Math.max(ij,jk),ki);
		}

		public int a,b,c;
		public float bbRound; // roundbounding box
		public String toString(){return a+","+b+","+c;}
	}
	
	public void update(int ms){
		
		//update pos
		System.out.println("posBefore: "+position);
		System.out.println("vitBefore: "+vitesse);
		calculF.set(vitesse).multLocal(ms/* *0.001f*/);
		position.addLocal(calcul1.set(calculF));
		calculF.set(lastAccel).multLocal(ms*ms/* *0.000001f*/);
		position.addLocal(calcul1.set(calculF));
//		position.x += (float)( (((double)vitesse.x)*ms)/1000 + (((((double)acceleration.x)*ms)/1000)*ms)/1000 );
//		position.y += (float)( (((double)vitesse.y)*ms)/1000 + (((((double)acceleration.y)*ms)/1000)*ms)/1000 );
//		position.z += (float)( (((double)vitesse.z)*ms)/1000 + (((((double)acceleration.z)*ms)/1000)*ms)/1000 );
		
		//update vitesse
		calculF.set(lastAccel).multLocal(ms/* *0.0005f*/ *0.5f);
		vitesse.addLocal(calculF);
		calculF.set(acceleration).multLocal(ms* /* 0.0005f*/ 0.5f);
		vitesse.addLocal(calculF);
//		vitesse.x += (float)( (((double)lastAccel.x)*ms)/2000 + (((double)acceleration.x)*ms)/2000 );
//		vitesse.y += (float)( (((double)lastAccel.y)*ms)/2000 + (((double)acceleration.y)*ms)/2000 );
//		vitesse.z += (float)( (((double)lastAccel.z)*ms)/2000 + (((double)acceleration.z)*ms)/2000 );
		
		Vector3f temp = lastAccel;
		lastAccel = acceleration;
		acceleration = temp;
		//acceleration.set(0,0,0);
		transfoMatrix.setTransform(position.toVec3f(), Vector3f.UNIT_XYZ, pangulaire.toRotationMatrix());
		//apply gravity?
		if(!landed){
			acceleration.addLocal(calculF.set(0, /* -9.81f*/ -0.00981f, 0));
			//System.out.println("not landed");
		}
//		System.out.println("vitesse:"+vitesse);
	}
	//apply a force in g*m/s2
	public void applyForce(Vector3d force) {
//		System.out.println("force = "+force);
		//transform
		force.toVec3fLocal(calculF).divideLocal((float) mass);
//		transfoMatrix.invert().multNormal(calculF, calculF);
//		System.out.println("force transformée = "+calculF);
		//add to accel
//		System.out.println("force transformée = "+calcul1.set(calculF));
		acceleration.addLocal(calculF);
//		System.out.println("now accel = "+acceleration);
	}
	
	public CollisionMobileSol checkCollision(Forme f2){
		if(f2.landed){
			if(!landed) return checkCollisionWithLand(f2);
		}else{
			if(landed){
				return f2.checkCollisionWithLand(this);
			}else{
				return checkCollisionWithFreeFlight(f2);
			}
		}
		return null;
	}
	
	//TODO
	public CollisionMobileSol checkCollisionWithFreeFlight(Forme f2){
		return null;
	}

	public CollisionMobileSol checkCollisionWithLand(Forme f2){
		if(!checkAlmostCollisionWithLand(f2)) return null;

		System.out.println("object1 trsf: "+transfoMatrix);
		System.out.println("object2 trsf: "+f2.transfoMatrix);
		
//		CollisionMobileSol test1 = checkExactCollision(f2, this.vitesse);
//		if(test1 == null) test1 = f2.checkExactCollision(this, calculF.set(vitesse).negateLocal());
//		if(test1 != null){
//			test1.mobile = this;
//			test1.sol = f2;
//		}
//		return test1;
		return null;
		//return checkAlmostCollisionWithLand(f2);
	}
	
//	public boolean checkExactCollision(Forme f2, Vector3f dir){
//		
////		if(CollisionMobileSol.detect(this, f2)){
////			return new CollisionMobileSol();
////		}
//	}

	// enfait, cette fonction check si collision, mais on peut rien en tirer de plus.
	// en fait, elle ne check meme pas correctement...
//	public CollisionMobileSol checkExactCollision(Forme f2, Vector3f dir){
//		System.out.println("checkExactCollision for "+this+" to "+f2);
//		//check by poly
//		Vector3f tempA = new Vector3f();
//		Vector3f tempB = new Vector3f();
//		Vector3f tempC = new Vector3f();
//		Vector3f tempP = new Vector3f();
//		Vector3f tempPInPlan = new Vector3f();
//
//		Plane plan = new Plane();
//		//get all poly 
//		
//		for(int idxTri=0;idxTri < triangles.size(); idxTri++){
//			Triangle tri = triangles.get(idxTri);
//			//create the plane for this triangle
//			plan.setPlanePoints(transfoMatrix.mult(points.get(tri.a), tempA), 
//					transfoMatrix.mult(points.get(tri.b), tempB),
//					transfoMatrix.mult(points.get(tri.c), tempC));
//			System.out.println("testing tri "+tri);
//			//check if it's on the good direction
//			if(plan.getNormal().dot(dir) > 0){
//				//transform it in the f2 repere
//				//check all closes points in the other forme
//				for(int idxPoint =0; idxPoint < f2.points.size(); idxPoint++){
//					Vector3f p = f2.points.get(idxPoint);
//					f2.transfoMatrix.mult(p, tempP);
//					System.out.println("testing with p "+p+" => "+plan.whichSide(f2.transfoMatrix.mult(p, tempP))+" ("+f2.transfoMatrix.mult(p, tempP)+")");
//					if(plan.whichSide(tempP) != Side.Positive){
//						//check if inside triangle
//						plan.getClosestPoint(tempP, tempPInPlan);
//						//'near' the triangle?
//						System.out.println("near? "+tempA.distance(tempPInPlan)+" =?= "+tri.bbRound);
//						if(tempA.distance(tempPInPlan) < tri.bbRound){
//							//ok, do the math
//							//move inside the new repere
//							
//							// Compute vectors        
//							Vector3f v0 = tempC.subtract(tempA);
//							Vector3f v1 = tempB.subtract(tempA); 
//							Vector3f v2 = tempPInPlan.subtract(tempA);
//
//							// Compute dot products
//							float dot00 = v0.dot(v0);
//							float dot01 = v0.dot(v1);
//							float dot02 = v0.dot(v2);
//							float dot11 = v1.dot(v1);
//							float dot12 = v1.dot(v2);
//
//							// Compute barycentric coordinates
//							float invDenom = 1 / (dot00 * dot11 - dot01 * dot01);
//							float u = (dot11 * dot02 - dot01 * dot12) * invDenom;
//							float v = (dot00 * dot12 - dot01 * dot02) * invDenom;
//
//							// Check if point is in triangle
//							System.out.println("inside? u="+u+", v="+v+", u+v="+(u+v));
//							boolean inside = (u >= 0) && (v >= 0) && (u + v < 1);
//							if(inside){
//								CollisionMobileSol obj = new CollisionMobileSol();
//								obj.formePointe = f2;
//								obj.formeTriangle = this;
//								obj.idxPointe = idxPoint;
////								obj.idxTriangle = idxTri;
////								obj.positionInWorldPos = tempPInPlan;
////								obj.planTriangle = plan;
//								return obj;
//							}
//						}
//					}
//				}
//			}
//			
//		}
//		
//		return null;
//	}
	
	public boolean checkAlmostCollisionWithLand(Forme f2){

		Vector3d directionObj1 = new Vector3d().set(vitesse);
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
	
	public Vector3f getMostFarAwayPoint(Vector3f direction, Vector3f returnVal){

		//get our more far away point in moving direction

		transfoMatrix.invert().multNormal(calculF.set(direction), calculF).normalizeLocal();
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
//		System.out.println("use point "+farAway);
		transfoMatrix.mult(farAway, calculF);
//		System.out.println("in world space: "+calculF);
		
		return returnVal.set(calculF);
	}
	
	
	//like init method
	//set tri to not face the center
	public void doNotFaceCenter() {
//		Vector3f tempA = new Vector3f();
//		Vector3f tempB = new Vector3f();
//		Vector3f tempC = new Vector3f();
		Vector3f tempP = new Vector3f(0,0,0);

		Plane plan = new Plane();
		for(Triangle tri : triangles){
//			plan.setPlanePoints(transfoMatrix.mult(points.get(tri.a), tempA), 
//			transfoMatrix.mult(points.get(tri.b), tempB),
//			transfoMatrix.mult(points.get(tri.c), tempC));	//create plane
			plan.setPlanePoints((points.get(tri.a)), 
			(points.get(tri.b)),
			(points.get(tri.c)));	//create plane
			if(plan.whichSide(tempP) == Side.Positive){
				System.out.println("Forme "+this+" : wrong side for triangle "+tri.a+tri.b+tri.c);
				int temp = tri.b;
				tri.b = tri.a;
				tri.a = temp;
			}
		}
		
		//update trsf matrix
		transfoMatrix.setTransform(position.toVec3f(), Vector3f.UNIT_XYZ, pangulaire.toRotationMatrix());
	}
	
}
