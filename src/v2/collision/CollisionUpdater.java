package v2.collision;

import java.util.ArrayList;
import java.util.Collection;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Iterator;
import java.util.Map.Entry;
import java.util.Set;
import java.util.TreeMap;

import jme3Double.Vector3d;
import v2.Forme;
import v2.Triangle;
import v2.collision.CollisionPrediction.Precision;

public class CollisionUpdater {

	// TreeMap<Long, Forme> movingObjects = new TreeMap<>();
	Collection<Forme> movingObjects = new HashSet<>();
	Collection<Forme> allObjects = new ArrayList<>();

	TreeMap<Long, CollisionPrediction> collisionDetected = new TreeMap<>();

	
	//working vars
	CollisionRayonTri colliderRT = new CollisionRayonTri();
	
	// 1 : new turn, redo all collision prediction
	public void redoCollisionPrediction(long time, long dtms) {
		// clean
		collisionDetected.clear();
		for (Forme f2 : allObjects) {
			f2.predictions.clear();
		}

		// maj
		for (Forme f1 : movingObjects) {
			System.out.println("check forme " + f1 + " : " + f1.physicUpdate + f1.position);
			if (f1.physicUpdate) {
				for (Forme f2 : allObjects) {
					if (f1 == f2)
						continue;
					// first lazy check (TODOAFTER: check if it's really useful)
					long timeCollide = canCollide(f1, f1.time, dtms + time - f1.time, f2, f2.time, dtms + time
							- f2.time);
					if (timeCollide > 0) {

						System.out.println("possible collision " + f1.predictions.get(f2));
						// check if collision exist (and it's not a joint)
						CollisionPrediction collider = f1.predictions.get(f2);
						if (collider == null /* && !f1.joint.containsKey(f2) */) {
							System.out.println("poCreate ssible collision ");
							// create a collisionPrediction
							collider = new CollisionPrediction();
							collider.moment = timeCollide;
							collider.precisionMoment = Precision.INITIAL;
							collider.maxMomentError = (f1.position.distance(f2.position) + f1.roundBBRayon + f2.roundBBRayon)
									/ (f1.getCachedVitesse(f1.time) + f2.getCachedVitesse(f2.time));
							collider.forme1 = f1;
							collider.forme2 = f2;
							f1.predictions.put(f2, collider);
							f2.predictions.put(f1, collider);
							collisionDetected.put(timeCollide, collider);
						}

					}
				}
			}
		}
	}
	
	//2: get the first collision
	public void resolveAllCollisionPrediction(long time, long dtms) {
		while(!collisionDetected.isEmpty()){
			Entry<Long, CollisionPrediction> predEntry = collisionDetected.pollFirstEntry();
//			Entry<Long, CollisionPrediction> nextEntry = collisionDetected.firstEntry();

			// check if this collision occur before any one else
			boolean first = true;
			for(Entry<Long, CollisionPrediction> pred2Check : collisionDetected.entrySet()){
				if(predEntry.getKey()+predEntry.getValue().maxMomentError >= 
						pred2Check.getKey()-pred2Check.getValue().maxMomentError){
					first = false;
					break;
				}
			}
			
			//if first, then we can resolve this collision
			if(first){
				resolveCollisionPrediction(predEntry.getValue(), time, dtms);
			}else{
				//sinon, il faut améliorer la précision des différentes collisions
				HashMap<Long, CollisionPrediction> toReAdd = new HashMap<>();
				Iterator<Entry<Long, CollisionPrediction>> it = collisionDetected.entrySet().iterator();
				while( it.hasNext() ){
					Entry<Long, CollisionPrediction> pred2Check  = it.next();
					//si la précision des autres m'empeche de me considérer premier
					if(predEntry.getKey() >= 
							pred2Check.getKey()-pred2Check.getValue().maxMomentError
							&& pred2Check.getValue().precisionMoment == Precision.INITIAL){
						long newTimePred = increaseCollisionPrecision(pred2Check.getValue(), time, dtms);
						if(pred2Check.getValue().precisionMoment == Precision.CANTFIND) 
							toReAdd.put(newTimePred,pred2Check.getValue());
						it.remove();
					}
				}
				collisionDetected.putAll(toReAdd);
				if(toReAdd.isEmpty()){
					//si ma précision est le problème
					long newTimePred = increaseCollisionPrecision(predEntry.getValue(), time, dtms);
					if(predEntry.getValue().precisionMoment == Precision.CANTFIND) 
						collisionDetected.put(newTimePred, predEntry.getValue());
				}else{
					collisionDetected.put(predEntry.getKey(), predEntry.getValue());
				}
			}
		}
	}
	
	//return new time for collision
	public long increaseCollisionPrecision(CollisionPrediction pred, long time, long dtms){
		if(pred.precisionMoment == Precision.INITIAL){
			//check with rayons and triangle BB (?)
			colliderRT.improvePrediction(pred, time, dtms);
		}else{
			//integrate ?
			//TODO
			//i think it's costly to integrate each duplet séparément
			// pourquoi ne pas intégrer toutes els collisions possible en meme temps?
		}
		return 0;
	}
	
	//integration:
	//1, trouver un ensemble de formes qui peuvent etre en collision.
	//2, intégrer jusqu'a ce qu'une ou des collisions arrive*
	//3, refaire la dernière intégration moins vite si il a une forme qui a touché plusieurs autres
	//4, maintenant on peut intégrer les duplet qui ont collidé jusqu'a collision.
	
	
	public void resolveCollisionPrediction(CollisionPrediction pred, long time, long dtms) {
		
		
		
	}
	

	//return the worst case for time prediction
	static long canCollide(Forme f1, long timef1, long dtmsf1, Forme f2, long timef2, long dtmsf2) {
		if (f1 == f2)
			return -1;
		// first lazy check (TODOAFTER: check if it's really useful)
		double dist2Check = f1.roundBBRayon + f2.roundBBRayon
		// + f1.vitesse.length()*dtmsf1 + f2.vitesse.length()*dtmsf2;
				+ f1.getCachedVitesse(timef1) * dtmsf1 + f2.getCachedVitesse(timef2);
		if (f1.position.distanceSquared(f2.position) < dist2Check * dist2Check) {
			System.out.println("possible collision (dist² +vit²) " + f1 + " with " + f2);

			// /second lazy check : collide with AABB from pos, bb and vit
			// TODOAFTER: maybe not useful after the previous check?
			if (f1.getAABB(timef1, dtmsf1).collide(f2.getAABB(timef2, dtmsf2))) {
				System.out.println("possible collision (AABB) " + f1 + " with " + f2);

				// TODOAFTER: third check for line-line distance (more precise
				// than mesh but infinite...
				// http://math.stackexchange.com/questions/1036959/midpoint-of-the-shortest-distance-between-2-rays-in-3d

				// fourth check: BB with mesh from center + roundBB + vitesse
				// TODOAFTER check if really useful, line-line detection can be
				// sufficient.
				// TODOAFTER if i keep that, i can pre-check mesh-segment
				// segment-mesh.
				if (colideMesh(f1.getMeshBB(timef1, dtmsf1), f2.getMeshBB(timef2, dtmsf2))) {
					System.out.println("possible collision (mesh cylinder) " + f1 + " with " + f2);

					//for the time, use the line check algo : where in the line it's nearer
					// div  |{center;point}| by vitesse
					// then use the min of the 2
					
					// ie no time prediction
					return Math.min(timef1, timef2);
				}
			}
		}
		return -1;
	}

	public static final Triangle[] trianglesMeshColl = new Triangle[12];
	static {
		trianglesMeshColl[0] = new Triangle(null, 0, 1, 2);
		trianglesMeshColl[1] = new Triangle(null, 0, 2, 3);
		trianglesMeshColl[2] = new Triangle(null, 4, 5, 6);
		trianglesMeshColl[3] = new Triangle(null, 4, 6, 7);
		trianglesMeshColl[4] = new Triangle(null, 0, 1, 5);
		trianglesMeshColl[5] = new Triangle(null, 0, 5, 4);
		trianglesMeshColl[6] = new Triangle(null, 1, 2, 6);
		trianglesMeshColl[7] = new Triangle(null, 1, 6, 5);
		trianglesMeshColl[8] = new Triangle(null, 2, 3, 7);
		trianglesMeshColl[9] = new Triangle(null, 2, 7, 6);
		trianglesMeshColl[10] = new Triangle(null, 3, 0, 4);
		trianglesMeshColl[11] = new Triangle(null, 3, 4, 7);
	}

	static boolean colideMesh(Vector3d[] verticesF1, Vector3d[] verticesF2) {
		// check the 144 possible collisions
		for (int i = 0; i < 12; i++) {
			for (int j = 0; j < 12; j++) {
				if (Triangle.collide(verticesF1[trianglesMeshColl[i].a], verticesF1[trianglesMeshColl[i].b],
						verticesF1[trianglesMeshColl[i].c],

						verticesF2[trianglesMeshColl[j].a], verticesF2[trianglesMeshColl[j].b],
						verticesF2[trianglesMeshColl[j].c])) {
					return true;
				}
			}
		}
		return false;
	}

}
