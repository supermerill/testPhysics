package old;

import com.jme3.light.PointLight;
import com.jme3.math.Plane;
import com.jme3.math.Ray;
import com.jme3.math.Vector3f;

public class CollisionMobileSol {
	Forme mobile;
	Forme sol;
	Forme formePointe;
	int idxPointe; //pas forcément bon, c'est le premier rencontré.
	Forme formeTriangle;
	int idxTriangle; //pas bon
	Plane planTriangle; //pas bon, c'en est un au pif
	int idxNewPointe;
	Vector3f positionInWorldPos; // truc useless : projection de la pointe sur
									// le triangle

	// deplace le mobile pour le mettre au bon endroit.
	void placeMobile(){
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
		TODO
		
		//calcul translation a faire.

		//prendre la vitesse du mobile
		//trouver l'endroit ou la pointe traverse le plan selon l'axe de la vitesse
		Vector3f pointIntersect = new Vector3f();
		Vector3f pointPointe = new Vector3f();
		formePointe.transfoMatrix.mult(formePointe.points.get(idxPointe), pointPointe);
		System.out.println("=...................................=");
		System.out.println("pointTriangleA="+formeTriangle.transfoMatrix.mult(formeTriangle.points.get(formeTriangle.triangles.get(idxTriangle).a), pointIntersect));
		System.out.println("pointTriangleB="+formeTriangle.transfoMatrix.mult(formeTriangle.points.get(formeTriangle.triangles.get(idxTriangle).b), pointIntersect));
		System.out.println("pointTriangleC="+formeTriangle.transfoMatrix.mult(formeTriangle.points.get(formeTriangle.triangles.get(idxTriangle).c), pointIntersect));
		System.out.println("Pointe="+pointPointe);
		System.out.println("mobile.vitesse="+mobile.vitesse);
		System.out.println("planTriangle="+planTriangle);

		//calculer la différence entre la pointe actuelle et "sa postion voulue"
		Vector3f pointCalcul = new Vector3f();

		if (formePointe == mobile) {
			new Ray(pointPointe, mobile.vitesse.negate()).intersectsWherePlane(planTriangle, pointIntersect);
			System.out.println("pointIntersect(mobile)="+pointIntersect);
			pointIntersect.subtract(pointCalcul, pointCalcul);
		} else {
			boolean result = new Ray(pointPointe, mobile.vitesse).intersectsWherePlane(planTriangle, pointIntersect);
			System.out.println(result+" pointIntersect(sol)="+pointIntersect);
			new Ray(pointPointe, mobile.vitesse).intersectWhere(
					formeTriangle.transfoMatrix.mult(formeTriangle.points.get(formeTriangle.triangles.get(idxTriangle).a), pointCalcul),
					formeTriangle.transfoMatrix.mult(formeTriangle.points.get(formeTriangle.triangles.get(idxTriangle).a),pointCalcul),
					formeTriangle.transfoMatrix.mult(formeTriangle.points.get(formeTriangle.triangles.get(idxTriangle).a),pointCalcul), 
							pointIntersect);
			System.out.println("pointIntersect(sol)="+pointIntersect);
			pointCalcul.subtractLocal(pointIntersect);
		}
		System.out.println("pointCalcul="+pointCalcul);
		
		//deplacer
		System.out.println("position before="+mobile.position);
		mobile.position.addLocal(pointCalcul.x, pointCalcul.y, pointCalcul.z);
		System.out.println("position after="+mobile.position);
		
		
	}

	// créer un point au meme endroit que l'autre
	void splitTriangle() {

	}

	// lie les deux formes par leur points respectifs
	void linkFormes() {

	}

}
