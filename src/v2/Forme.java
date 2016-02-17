package v2;
import java.util.ArrayList;
import java.util.HashMap;

import v2.collision.AABB;
import v2.collision.CollisionPrediction;

import jme3Double.Matrix4d;
import jme3Double.PlaneD;
import jme3Double.PlaneD.Side;
import jme3Double.Quaterniond;
import jme3Double.Vector3d;

public class Forme {
	
	public String name;
	public String toString(){ return name;}
	public Forme(String name){ this.name = name;}
	
	///---------- phys/colision
	public boolean physicUpdate = false;
	//TODOAFTER del this, put it elsewhere
	public HashMap<Forme, CollisionPrediction> predictions = new HashMap<>(2);
	public Vector3d positionGravite = Vector3d.ZERO; // center of the gravity field for us.
	//constanteGravite = 6.6734 E-11 N*m²/kg²
	//default constanteGravite: 1g with 100m rayon (note: 1N = 10kg)
//	public float constanteGraviteMasse = 98100f; //can be updated with our position in space, in N*m²
//	public float constanteGraviteMasse = 0.0981f; //can be updated with our position in space, in N*km²
	public double constanteGraviteMasse = 98100000000f; //can be updated with our position in space, in N*mm²
//	public float constanteGraviteStd = 98100f;//398000f; //can be updated with our position in space, in m3/s²
	public ArrayList<Vector3d> forces = new ArrayList<>(); //en N
	public ArrayList<Vector3d> pointApplicationForce = new ArrayList<>(); //world repere
	public ArrayList<Integer> idxpointApplicationForce = new ArrayList<>(); //choisir une des deux
//	public ArrayList<Vector3f> angularForces = new ArrayList<>();
	
	// cached data for colision detection
	public AABB cachedAABB = new AABB();
	public long cachedAABBtime = 0;
	public Vector3d[] cachedMeshBB = new Vector3d[8]; {for(int i=0;i<8;i++){cachedMeshBB[i] = new Vector3d(); }}
	public long cachedMeshBBtime = 0;
	public double cachedVitesseLength = 0;
	public long cachedVitesseLengthTime = 0;
	//-------------
	
	
	public boolean landed = true;
	//rayon of the boundingbox (en mm) 
	// => rayon de la sphere qui contient l'objet, ie distance du vertex le plus éloigné du centre de gravité.
	public double roundBBRayon=1;
	// rayon de la sphere qui contient le AABB qui contient la sphere de rayon roundBBRayon
	//ie roundBBRayon*1.42
//	public double roundAABBRayon=1.42;
	public double mass = 1; // en g
	//use an overly-simplified model of moment of inertia.
	// moment of intertia: boule = mr²*2/5 tige(rot extrem): mL²/3 (4mr²/3)
	// pavé (sur axe x): m*(y²+z²)/12
	// our own : m*r²/2 = m*L²/8 (toupdate)
	//TODOAFTER2 : use a better approximation, computed at creation/ great moments
	//			ie, more sphere, rod or pave?
	public float getIntertiaMoment(){ return (float)(mass * roundBBRayon*roundBBRayon/20); }
	
	//it's always 0,0,0 !
	//Vector3d gravityCenter = new Vector3d(0,0,0); // en mm
	
	public long time; //en ms
	//par rapport au centre de gravite
	public Vector3d position = new Vector3d(); // en mm
	public Vector3d vitesse = new Vector3d(); // en mm/ms
	public Vector3d acceleration = new Vector3d(); //en mm/ms*ms
	public Vector3d lastAccel = new Vector3d(); //en mm/ms*ms

//	public Vector3f posAxeRot = new Vector3f(0,0,0); //en mm, local?
	public Quaterniond pangulaire = new Quaterniond(Quaterniond.IDENTITY); //en rad
	public Vector3d vangulaire = new Vector3d(); //en rad/ms
	public Vector3d aangulaire = new Vector3d(); //en rad/ms*ms
	public Vector3d lastAangulaire = new Vector3d(); //en rad/ms*ms

	public Matrix4d transfoMatrix = new Matrix4d();
	
	

	//mesh, en coordonees locale (besoin de passer par transfoMatrix)
	public ArrayList<Vector3d> points = new ArrayList<>();
	public ArrayList<Vector3d> normales = new ArrayList<>();
	public ArrayList<Triangle> triangles = new ArrayList<>();
	
	//temp vars (remove this alter)
	public Vector3d calcul1 = new Vector3d();
//	public Vector3f calculF = new Vector3f();
	
	public Vector3d getMostFarAwayPoint(Vector3d direction, Vector3d returnVal){

		//get our more far away point in moving direction

		transfoMatrix.invert().multNormal(direction, calcul1).normalizeLocal();
		//System.out.println("direction: "+calcul1);
		Vector3d farAway = null;
		double score = 0;
		for(Vector3d p : points){
			double newScore = calcul1.dot(p);
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
		transfoMatrix.mult(farAway, calcul1);
		System.out.println("in world space: "+calcul1);
		
		return returnVal.set(calcul1);
	}
	
	
	//like init method
	//set tri to not face the center
	public void doNotFaceCenter() {
//		Vector3f tempA = new Vector3f();
//		Vector3f tempB = new Vector3f();
//		Vector3f tempC = new Vector3f();
		Vector3d tempP = new Vector3d(0,0,0);

		PlaneD plan = new PlaneD();
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
		transfoMatrix.setTransform(position, Vector3d.UNIT_XYZ, pangulaire.toRotationMatrix());
	}
	
	public void addPoint(Vector3d point){
		points.add(point);
		normales.add(new Vector3d(point));
		System.out.println("point "+(points.size()-1)+": "+normales.get(points.size()-1));
	}
	
	public void computeNormales(){
		if(normales.size() != points.size()){ System.err.println("Error, not enough normales!"); System.exit(1);}
		PlaneD p = new PlaneD();
		int[] nbCompo = new int[points.size()];
		for(Triangle tri : triangles){
			//compute normale
			p.setPlanePoints(points.get(tri.a), points.get(tri.b), points.get(tri.c));
			normales.get(tri.a).addLocal(p.getNormal());
			nbCompo[tri.a]++;
			normales.get(tri.b).addLocal(p.getNormal());
			nbCompo[tri.b]++;
			normales.get(tri.c).addLocal(p.getNormal());
			nbCompo[tri.c]++;
		}
		for(int i=0;i<normales.size();i++){
			if(nbCompo[i]>0){
				normales.get(i).subtractLocal(points.get(i)).divideLocal(nbCompo[i]).normalizeLocal();
			}
		}
	}

	public AABB getAABB(long timeDeb, long dtms){
		if(cachedAABBtime != timeDeb){
			cachedAABB.set(position, roundBBRayon, vitesse, dtms);
			cachedAABBtime = timeDeb;
		}
		return cachedAABB;
	}
	
	//TODOAFTER: verify if correct (visually)
	public Vector3d[] getMeshBB(long timeDeb, long dtms){
		if(cachedMeshBBtime != timeDeb){
			//create bb
			//on cree le plan arriere
			PlaneD plan = new PlaneD();
			Vector3d normale = vitesse.normalize();
			plan.setOriginNormal(Vector3d.ZERO, normale);
			Vector3d dir1 = null;
			if(normale.y==0){
				dir1 = plan.getClosestPoint(Vector3d.UNIT_Y);
			}else{
				dir1 = plan.getClosestPoint(Vector3d.UNIT_X);
			}
			dir1.normalizeLocal();
			Vector3d dir2 = dir1.cross(normale);
			dir2.normalizeLocal();
			dir1.multLocal(roundBBRayon*1.42); //because rayon c'est el coté du carré, pas la diagonale.
			dir2.multLocal(roundBBRayon*1.42); 
			Vector3d pointPlan = normale.mult(roundBBRayon*-1.42).addLocal(position);
			cachedMeshBB[0].set(pointPlan).addLocal(dir1).addLocal(dir2);  // _|
			cachedMeshBB[1].set(pointPlan).addLocal(dir1).subtractLocal(dir2);	// -|
			cachedMeshBB[2].set(pointPlan).subtractLocal(dir1).subtractLocal(dir2);	// |-
			cachedMeshBB[3].set(pointPlan).subtractLocal(dir1).addLocal(dir2);  // |_
			pointPlan.addLocal(vitesse.mult(dtms)).addLocal(normale.mult(roundBBRayon*2.84));
			cachedMeshBB[4].set(pointPlan).addLocal(dir1).addLocal(dir2);  // _|
			cachedMeshBB[5].set(pointPlan).addLocal(dir1).subtractLocal(dir2);	// -|
			cachedMeshBB[6].set(pointPlan).subtractLocal(dir1).subtractLocal(dir2);	// |-
			cachedMeshBB[7].set(pointPlan).subtractLocal(dir1).addLocal(dir2);  // |_
//			roundBBRayon*1.42
			cachedMeshBBtime = timeDeb;
		}
		return cachedMeshBB;
	}

	public double getCachedVitesse(long timeDeb){
		if(cachedVitesseLengthTime != timeDeb){
			cachedVitesseLengthTime = timeDeb;
			cachedVitesseLength = vitesse.length();
		}
		return cachedVitesseLength;
	}
}
