package old;

import collision.CollisionPrediction;

import com.jme3.light.PointLight;
import com.jme3.math.Plane;
import com.jme3.math.Ray;
import com.jme3.math.Vector3f;

@Deprecated
public class CollisionMobileSol {
	Forme mobile;
	Forme sol;
	Forme formePointe;
	int idxPointe;
	Forme formeTriangle;
	int idxTriangle;
	int idxNewPointe;
	Vector3f positionInWorldPos;
	
	static public boolean detect(Forme mobile, Forme sol){
		Vector3f tempA = new Vector3f();
		Vector3f tempB = new Vector3f();
		Vector3f tempC = new Vector3f();
		Vector3f tempP = new Vector3f();
		Vector3f tempCalc = new Vector3f();
		Vector3f bestP = new Vector3f();
		Forme bestPointeur = null;
		Forme bestFormeTri = null;
		float distMin = 10000000;
		int bestIdxTri = -1;
		int bestIdxPoint = -1;
		Ray ray = new Ray();
		//create new dir
		Vector3f vitesseF1 = mobile.vitesse.negate();
		ray.setDirection(vitesseF1);
		System.out.println("--------------- rayCollision ---------- "+vitesseF1);
		Forme pointeur = mobile;
		Forme formeTri = sol;
		//check pointe from f1
		for(int idxTri =0; idxTri < formeTri.points.size(); idxTri++){
			Triangle tri = formeTri.triangles.get(idxTri);
			//create the plane for this triangle
			formeTri.transfoMatrix.mult(formeTri.points.get(tri.a), tempA);
			formeTri.transfoMatrix.mult(formeTri.points.get(tri.a), tempB);
			formeTri.transfoMatrix.mult(formeTri.points.get(tri.a), tempC); 
			for(int idxPoint =0; idxPoint < pointeur.points.size(); idxPoint++){
				pointeur.transfoMatrix.mult(pointeur.points.get(idxPoint), tempP);
				ray.setOrigin(tempP);
				
				if(ray.intersectWhere(tempA, tempB, tempC, tempCalc)){
					float dist = (tempCalc.distance(tempP));
					if(dist < distMin){
						System.out.println("find a futur intersec on "+tempCalc+ "@"+(tempCalc.distance(tempP)+" in tri "+idxTri));
						System.out.println("from "+ray.getOrigin()+ " with dir " + ray.getDirection());
						distMin = dist;
						bestP.set(tempCalc);
						bestPointeur = pointeur;
						bestFormeTri = formeTri;
						bestIdxTri = idxTri;
						bestIdxPoint = idxPoint;
					}
				}
			}
		}
		System.out.println("--------------- rayCollision2 ---------- "+vitesseF1.negate());
		//check pointe from f2
		ray.setDirection(vitesseF1.negate());
		pointeur = sol;
		formeTri = mobile;
		for(int idxTri =0; idxTri < formeTri.points.size(); idxTri++){
			Triangle tri = formeTri.triangles.get(idxTri);
			//create the plane for this triangle
			formeTri.transfoMatrix.mult(formeTri.points.get(tri.a), tempA);
			formeTri.transfoMatrix.mult(formeTri.points.get(tri.b), tempB);
			formeTri.transfoMatrix.mult(formeTri.points.get(tri.c), tempC);
//			System.out.println("testing tri "+tempA+ ", "+tempB+ ", "+tempC);
			for(int idxPoint =0; idxPoint < pointeur.points.size(); idxPoint++){
				pointeur.transfoMatrix.mult(pointeur.points.get(idxPoint), tempP);
				ray.setOrigin(tempP);
				//System.out.println("testing with orig: "+tempP+" : "+ray.intersects(tempA, tempB, tempC));
				
				if(ray.intersectWhere(tempA, tempB, tempC, tempCalc)){
					float dist = (tempCalc.distance(tempP));
					if(dist < distMin){
						System.out.println("find a futur intersec on "+tempCalc+ "@"+(tempCalc.distance(tempP)+" in tri "+idxTri));
						System.out.println("from "+ray.getOrigin()+ " with dir " + ray.getDirection());
						distMin = dist;
						bestP.set(tempCalc);
						bestPointeur = pointeur;
						bestFormeTri = formeTri;
						bestIdxTri = idxTri;
						bestIdxPoint = idxPoint;
					}
				}
			}
		}
		System.out.println("--------------- END rayCollision ---------- ");
		if(bestPointeur != null){
			CollisionPrediction pred =new CollisionPrediction();
			pred.formePoint = bestPointeur;
			pred.formeTri = bestFormeTri;
			pred.pointIdx = bestIdxPoint;
			pred.triIdx = bestIdxTri;
			//compute time
//			pred.moment = currentTime + (long)(1000 * distMin / vitesseF1.length());
			return true;
		}
		return false;
	}

	// deplace le mobile pour le mettre au bon endroit.
	void placeMobile(){
		Vector3f pointCalcul = new Vector3f();
		Vector3f pointPointe = new Vector3f();
		formePointe.transfoMatrix.mult(formePointe.points.get(idxPointe), pointPointe);
		//transalation ou rotation?
		//calcul de la vitesse de la rotation sur ce point
		//distance au centre de gravite
//		Vector3f positionInMobilePos = mobile.transfoMatrix.invert().mult(positionInWorldPos, new Vector3f());
//		float vitesseAngulaireRadS = mobile.vangulaire.norm(); //in rad/s
//		float distancePoint = positionInMobilePos.length(); //in m
//		//1 rad/s , rayon =1m => pi/2 m/s
//		float vitesseFromRotation
		//TODO placer /\ ca dans la detection, pas (forcement)
		
		//chercher le triangle transpercé.
		//prendre la vitesse comme vecteur du rayon (ou l'opposé si la pointe est sur le mobile).
		//prendre la position de la pointe comme le départ
		//lancer de rayon pour chaque triangle de la formetriangle, on garde que la position du résultat le plus proche de la pointe.
		//celui que l'on a gardé est le bon triangle (et on a la position pour le reste du calcul).
		Vector3f directionLancer = new Vector3f(mobile.vitesse);
		if (formePointe == mobile) {
			System.out.println("formePointe== mobile");
			directionLancer.negateLocal();
		}
		Ray ray = new Ray(pointPointe, directionLancer);
		Vector3f tempVector = new Vector3f(0,0,0);
		Vector3f bestCollisionPoint = new Vector3f(0,0,0);
		float bestDist = 1000000000;
		System.out.println("Testing with Vector="+directionLancer);
		System.out.println("Testing from="+pointPointe);
		for(int idxTri=0;idxTri < formeTriangle.triangles.size(); idxTri++){
			Triangle tri = formeTriangle.triangles.get(idxTri);
			System.out.println("Testing with tri="+idxTri+" : "+tri);
			System.out.println("Testing with tri="+formeTriangle.transfoMatrix.mult(formeTriangle.points.get(tri.a), pointCalcul)+","
					+formeTriangle.transfoMatrix.mult(formeTriangle.points.get(tri.b),pointCalcul)+","
					+formeTriangle.transfoMatrix.mult(formeTriangle.points.get(tri.c),pointCalcul)
					);
			System.out.println("intersects:"+ray.intersects(
					formeTriangle.transfoMatrix.mult(formeTriangle.points.get(tri.a),pointCalcul),
					formeTriangle.transfoMatrix.mult(formeTriangle.points.get(tri.b), pointCalcul),
					formeTriangle.transfoMatrix.mult(formeTriangle.points.get(tri.c),pointCalcul)));
			if(ray.intersectWhere(
					formeTriangle.transfoMatrix.mult(formeTriangle.points.get(tri.a),pointCalcul),
					formeTriangle.transfoMatrix.mult(formeTriangle.points.get(tri.b), pointCalcul),
					formeTriangle.transfoMatrix.mult(formeTriangle.points.get(tri.c),pointCalcul)
					,tempVector)){
				//check if better
				float dist = pointPointe.distance(tempVector);
				if(dist < bestDist){
					bestDist = dist;
					bestCollisionPoint.set(tempVector);
					idxTriangle = idxTri;
					positionInWorldPos = bestCollisionPoint;
				}
			}else{
				System.out.println("not found "+tempVector);
			}
		}
		//check if tri finded
		if(bestDist >= 1000000000){
			//error no collision?!?
			System.err.println("CollisionMobileSol: error, no collision detected but detected....");
			return;
		}
		
		
		//calcul translation a faire.

		//prendre la vitesse du mobile
		//trouver l'endroit ou la pointe traverse le plan selon l'axe de la vitesse
		Vector3f pointIntersect = new Vector3f();
		System.out.println("=...................................=");
		System.out.println("pointTriangleA="+formeTriangle.transfoMatrix.mult(formeTriangle.points.get(formeTriangle.triangles.get(idxTriangle).a), pointIntersect));
		System.out.println("pointTriangleB="+formeTriangle.transfoMatrix.mult(formeTriangle.points.get(formeTriangle.triangles.get(idxTriangle).b), pointIntersect));
		System.out.println("pointTriangleC="+formeTriangle.transfoMatrix.mult(formeTriangle.points.get(formeTriangle.triangles.get(idxTriangle).c), pointIntersect));
		System.out.println("Pointe="+pointPointe);
		System.out.println("mobile.vitesse="+mobile.vitesse);
//		System.out.println("planTriangle="+planTriangle);

		//calculer la diff�rence entre la pointe actuelle et "sa postion voulue"

//		if (formePointe == mobile) {
//			new Ray(pointPointe, mobile.vitesse.negate()).intersectsWherePlane(planTriangle, pointIntersect);
//			System.out.println("pointIntersect(mobile)="+pointIntersect);
//			pointIntersect.subtract(pointCalcul, pointCalcul);
//		} else {
//			boolean result = new Ray(pointPointe, mobile.vitesse).intersectsWherePlane(planTriangle, pointIntersect);
//			System.out.println(result+" pointIntersect(sol)="+pointIntersect);
//			new Ray(pointPointe, mobile.vitesse).intersectWhere(
//					formeTriangle.transfoMatrix.mult(formeTriangle.points.get(formeTriangle.triangles.get(idxTriangle).a), pointCalcul),
//					formeTriangle.transfoMatrix.mult(formeTriangle.points.get(formeTriangle.triangles.get(idxTriangle).b),pointCalcul),
//					formeTriangle.transfoMatrix.mult(formeTriangle.points.get(formeTriangle.triangles.get(idxTriangle).c),pointCalcul), 
//							pointIntersect);
//			System.out.println("pointIntersect(sol)="+pointIntersect);
//			pointCalcul.subtractLocal(pointIntersect);
//		}
		pointCalcul.set(pointPointe);
		System.out.println("pointCalcul(pointe)="+pointCalcul);
		System.out.println("bestCollisionPoint="+bestCollisionPoint);
		if (formePointe == mobile) {
			bestCollisionPoint.subtract(pointCalcul, pointCalcul);
		}else{
			pointCalcul.subtractLocal(bestCollisionPoint);
		}
		System.out.println("pointCalcul(move)="+pointCalcul);
		//deplacer
		System.out.println("position before="+mobile.position);
		mobile.position.addLocal(pointCalcul.x, pointCalcul.y, pointCalcul.z);
		System.out.println("position after="+mobile.position);
		
		
	}

	// cr�er un point au meme endroit que l'autre
	void splitTriangle() {

	}

	// lie les deux formes par leur points respectifs
	void linkFormes() {

	}

}
