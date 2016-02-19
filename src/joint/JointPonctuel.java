package joint;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;

import jme3Double.Vector3d;

import com.jme3.math.FastMath;
import com.jme3.math.Matrix4f;
import com.jme3.math.Plane;
import com.jme3.math.Quaternion;
import com.jme3.math.Ray;
import com.jme3.math.Vector3f;

import old.Forme;

public class JointPonctuel extends JointRotation {

	// del these if not used.
	// public Forme f1;
	// public int idxPointf1;
	// public Forme f2;
	// public int idxPointf2;

	// the point-joint
	// public Vector3f point; //World coord
	public int idxPoint;
	Forme fOpposite;
	public int idxOpposite;

	public boolean freeFlight = false;;

	public JointPonctuel(Forme f, Vector3f pointCollision, int idx, Forme fOpposite, int idxOpposite) {
		super(f);
		this.pointWRotation.set(pointCollision);
		this.idxPoint = idx;
		this.pointLRotation.set(f.points.get(idx));
		this.fOpposite = fOpposite;
		this.idxOpposite = idxOpposite;
		

		System.out.println("JPc new Joint ponctuel has: "+f+" : " + pointWRotation + ".distance("
				+f.transfoMatrix.mult(pointLRotation)
				+ ") ="+pointWRotation.distance(f.transfoMatrix.mult(pointLRotation))+" > 0.0001");
	}

	@Override
	public void updatePosition(long instant, long dt) {
		super.updatePosition(instant, dt);
		// recaler le solide
		System.out.println("BeforeRecalage : " + pointWRotation + ".distance("
				+ f.transfoMatrix.mult(f.points.get(idxPoint)) 
				+ ") ="+pointWRotation.distance(f.transfoMatrix.mult(new Vector3f(f.points.get(idxPoint)))));
		System.out.println("UpdatePos : recalage from "+f+"@"+f.position
				+" of "+pointWRotation.subtract(f.transfoMatrix.mult(new Vector3f(f.points.get(idxPoint)))));
		f.position.addLocal(pointWRotation.subtract(f.transfoMatrix.mult(new Vector3f(f.points.get(idxPoint)))));
		f.transfoMatrix.setTranslation(f.position.toVec3f());
		// check if it has moved (more than 0.1mm)
		if (freeFlight || pointWRotation.distance(f.transfoMatrix.mult(pointLRotation)) > 0.0001f) {
			f.joint = new JointFreeFlight(f);
		}
		System.out.println("AfterRecalage : " + pointWRotation + ".distance("
				+ f.transfoMatrix.mult(f.points.get(idxPoint)) 
				+ ") ="+pointWRotation.distance(f.transfoMatrix.mult(new Vector3f(f.points.get(idxPoint)))));
		
	}

	@Override
	public void updateForce(long instant, long dt) {
		if (f.landed)
			return;

		//for each force, decompose them into linear & angular
		System.out.println("JPc updateForce! "+f.forces.size());
		Vector3f sumLinear = new Vector3f(0,0,0);
		Vector3f sumAngular = new Vector3f(0,0,0);
		Vector3f fpos = f.position.toVec3f();
		Vector3f vectDir = new Vector3f();
		float sumPercent = 0;
		ArrayList<Float> percentAngular = new ArrayList<>();
		ArrayList<Vector3f> angularForce = new ArrayList<>();
		for(int i=0; i<f.forces.size(); i++){
			percentAngular.add(0f);
			angularForce.add(new Vector3f());
			vectDir.set(fpos).subtractLocal(f.pointApplicationForce.get(i)).normalizeLocal();
			if(vectDir.lengthSquared() == 0){
//				System.out.println("onlylinear : "+f.forces.get(i));
				sumLinear.addLocal(f.forces.get(i));
			}else{
				float length = f.forces.get(i).length();
				float dot = vectDir.dot(f.forces.get(i));
				sumLinear.addLocal(f.forces.get(i).mult(dot/length));
//				System.out.println("linear : "+vectDir.mult(dot));
				if(dot < length){ //+epsilon?
					Vector3f angularVect = f.forces.get(i).cross(vectDir);
					angularVect.multLocal((length-dot)/length);
//					System.out.println("angular : "+angularVect+" "+f.forces.get(i)+" cross "+vectDir+" mult "+((length-dot)/length));
					
					//recompose angular into linear if combine
					// note: it should not occur on a free-flight object.
					float maxLength = Math.min(length, sumAngular.length());
					if(maxLength>0){
						float percent = angularVect.normalize().dot(sumAngular.normalize());
						//quantity of linear to re-add *2 if this angular force isn't completely compensated
						float quantity = percent*(length-dot)/length;
						//quantity of linear to re-add *2 if this angular force is completely compensated
						if(percent*sumAngular.length() > quantity){
							quantity = length;
						}
						if(percent < 0){
//							System.out.println("addLinear : "+f.forces.get(i).mult(-2*percent*(length-dot)/length));
							//*2 because there are this force and the other force which compensate me
							sumLinear.addLocal(f.forces.get(i).mult(-2*quantity));
							
							//remove some from other angular forces
							float totPercentRemove = 0;
							for(int j=0; j<i; j++){
								totPercentRemove += angularVect.normalize().dot(angularForce.get(j).normalize());
							}
							for(int j=0; j<i; j++){
								float percentRemove = angularVect.normalize().dot(angularForce.get(j).normalize());
								percentRemove = percentRemove / totPercentRemove;
								float quantityRemove = quantity * percentRemove;
								percentAngular.set(j,  percentAngular.get(j) - quantityRemove);
								angularForce.set(j, angularForce.get(j).mult(1-percentRemove));
							}
							sumPercent -= quantity;
						}else{
							percentAngular.set(i, quantity);
							sumPercent += quantity;
							sumAngular.addLocal(angularVect);
							angularForce.set(i, angularVect);
						}
					}else{
						percentAngular.set(i, 1f);
						sumPercent += 1f;
						sumAngular.addLocal(angularVect);
						angularForce.set(i, angularVect);
					}
				}
			}
		}
		System.out.println("JPc linear intermediaire : "+sumLinear);
		System.out.println("JPc angular intermediaire : "+sumAngular);
		
		//now create the resultante on the contact point
		


		// create the normal force (via a dot)
//		f.acceleration.set(f.positionGravite).subtract(fpos);
//		sumLinear.divideLocal((float)f.mass);
//		f.acceleration.addLocal(sumLinear);
		
		//add gravity to sumlinear
		System.out.println("JPc sumLinear, before gravity : " + sumLinear + " (length=" + sumLinear.length()+")");

		Vector3f gravity = f.positionGravite.subtract(fpos);
		float distGravity = gravity.length();
		gravity.normalizeLocal().multLocal((float)(f.constanteGraviteMasse/(distGravity*distGravity*f.mass)));
//		sumLinear.addLocal(f.positionGravite.subtract(fpos).normalize().mult((float)(f.mass * f.constanteGraviteMasse)));
		sumLinear.addLocal(gravity);
		System.out.println("JPc Gravity = "+gravity);
		
		// normalized vector for RO
		Vector3f vectPointN = f.position.toVec3f().subtractLocal(pointWRotation);
		vectPointN.normalizeLocal();
		 // seems to be ???????
		Vector3f normal = vectPointN.mult(vectPointN.dot(sumLinear));
		 // seems to be the rotational vector from linear force if it push against pointWRot ?
		Vector3f angularFfromLinear = vectPointN.cross(sumLinear.normalize()).mult(sumLinear.length() - normal.length());
		
		System.out.println("JPc vectPointN : " + vectPointN);
		System.out.println("JPc length/totalLength : " + vectPointN.dot(sumLinear) + " / " + sumLinear.length());
		System.out.println("JPc sumLinear : " + sumLinear + " (length=" + sumLinear.length()+")");
		System.out.println("JPc normale : " + normal + " (length=" + normal.length());
		System.out.println("JPc angularFfromLinear : " + angularFfromLinear + " (length=" + angularFfromLinear.length()+")");
		
		
		if (normal.length() < 0) {
			// it's reversed!
			//freeFlight = true;
			
		} else {

			// create rotationVector
			/// ???
//			Vector3f rotVector = vectPointN.cross(sumLinear.normalize()).mult(sumLinear.length() - normal.length());
////			rotVector.divideLocal(f.getIntertiaMoment());
//			angularFfromLinear.set(rotVector);
//			f.aangulaire.addLocal(rotVector);
//			System.out.println("sum : " + sumLinear);
//			System.out.println("cross : " + normalN.cross(sumLinear.normalize()));
//			System.out.println("crossN : "
//					+ normalN.cross(sumLinear.normalize()).mult(sumLinear.length() - normal.length()));
//			System.out.println("rotVector : " + rotVector);
//			System.out.println("f.aangulaire : " + f.aangulaire);

		}
		
		//TODO FIXME linearAtRot & sumLinear -> need to 'add' them where?

		//create resultante on contact point for the angular force
		Vector3f linearAtRot = sumAngular.cross(pointWRotation.subtract(fpos));
		System.out.println("JPc sumAngular="+sumAngular+" X pointWRotation.subtract(fpos)="+pointWRotation.subtract(fpos)+" ==> linearAtRot="+linearAtRot);
//		Vector3f vect = fpos.subtract(this.pointWRotation);
		float checkOrientation = linearAtRot.dot(f.normales.get(this.idxPoint));
		System.out.println("JPc checkOrientation="+checkOrientation);
		Plane planRotation = new Plane(sumAngular, 0);
		if (checkOrientation >= 0) {
			System.out.println("JPc point @"+this.pointLRotation+" is in the right direction for angular pivot");
			//check dist  edit: ????
			float val = planRotation.getClosestPoint(fpos.subtract(this.pointWRotation)).lengthSquared();
			if(val > 0){
				//choke point!
				//add linear to sum linear
				//TODOAFTER: integrate the linear compo better than that! (to not move the point of rotation)

//				// il y a le point d'application de la force F1 en F
//				// il y a la force contre laquel on lutte (oupas?) appliqué en C
//				// il y a le point pivot de rotation P
//				// => il y a une force F2 en C de taille |F1|*FP/PC et de direction -F1

				Vector3f newSumAngular = new Vector3f(0,0,0);
				for(int i=0; i<f.forces.size(); i++){
					if(percentAngular.get(i)>0){
						float FP = pointWRotation.subtract(f.pointApplicationForce.get(i)).length();
						float PC = pointWRotation.subtract(fpos).length();
						newSumAngular.addLocal(angularForce.get(i).mult(FP/PC));
					}
				}
				
				// ????
				if (normal.length() > 0){
					//it can't be freeflight
					newSumAngular.addLocal(angularFfromLinear);
					newSumAngular.divideLocal(f.getIntertiaMoment());
					f.aangulaire.set(newSumAngular);
					System.out.println("JPc LEVIER+FORCE");
				}else{
					//it can be free flight
//					Vector3f linearAtRot = sumAngular.cross(pointWRotation.subtract(fpos));
//					Vector3f isFreeflight = linearAtRot.subtract(normal);
//					if (isFreeflight.dot(linearAtRot.add(sumLinear)) > 0) {
					if(linearAtRot.add(sumLinear).length() > 0){
						System.out.println("JPc LEVIER-FORCE => FREE");
//						freeFlight = true;
						fOpposite.joint.removeCollisionPoint(pointWRotation, idxOpposite);
						f.joint = new JointFreeFlight(f);
						f.joint.updateForce(instant, dt);
						//en principe, changement de centre de rotation
						// =>le moment change 
						// => changement de la vitesse angulaire
						//TODOAFTER
//						
						//freeFlight = true;
					}else{
						System.out.println("JPc LEVIER-FORCE => ROT");
						//pareil, en principe le moment n'est pas bon...?
						newSumAngular.addLocal(angularFfromLinear);
						newSumAngular.divideLocal(f.getIntertiaMoment());
						f.aangulaire.addLocal(sumAngular);
					}
				}
				
			}//ELSE?????? FIXME?
		}else{
			System.out.println("JPc point @"+this.pointLRotation+" is not in the right direction for angular pivot");
			//=> ajouter en tant que force de rotation "free flight"
			
			//TODO: this rotation is not in pointWRotation !! 
//			sumAngular.divideLocal(f.getIntertiaMoment());
//			f.aangulaire.set(sumAngular);
			
			//check if it's free flight or not
//			Vector3f linearAtRot = sumAngular.cross(pointWRotation.subtract(fpos));
			System.out.println("sumAngular="+sumAngular+", pointWRotation.subtract(fpos)="+pointWRotation.subtract(fpos));
//			Vector3f isFreeflight = linearAtRot.subtract(normal); /// ????
//			System.out.println(isFreeflight+" dot "+linearAtRot+".add("+sumLinear+") = "+isFreeflight.dot(linearAtRot.add(sumLinear))+ " >? 0");
			System.out.println("linearAtRot="+linearAtRot+", sumLinear="+sumLinear);
			if (linearAtRot.add(sumLinear).length() > 0) {
				System.out.println("JPc -LEVIER-+FORCE => FREE");
//				freeFlight = true;
				fOpposite.joint.removeCollisionPoint(pointWRotation, idxOpposite);
				f.joint = new JointFreeFlight(f);
				f.joint.updateForce(instant, dt);
				//en principe, changement de centre de rotation
				// =>le moment change 
				// => changement de la vitesse angulaire
				//TODOAFTER
//				
				//freeFlight = true;
			}else{
				System.out.println("JPc -LEVIER-+FORCE => ROT");
				//pareil, en principe le moment n'est pas bon...
				sumAngular.addLocal(angularFfromLinear);
				sumAngular.divideLocal(f.getIntertiaMoment());
				f.aangulaire.addLocal(sumAngular);
			}
			
		}
		System.out.println("JPc f.aangulaire="+f.aangulaire);
		
		
		

//		f.aangulaire.set(0, 0, 0);
//		// compute sum of force
//		Vector3f sumForce = new Vector3f(0, 0, 0);
//		for (Vector3f force : f.forces) {
//			sumForce.addLocal(force);
//		}

		// Angular force: just transpose them to angular acceleration
//		Vector3f sumAngular = new Vector3f(0, 0, 0);
//		for (Vector3f force : f.angularForces) {
//			sumAngular.addLocal(force);
//		}
//		sumAngular.divideLocal(f.getIntertiaMoment());
//		f.aangulaire.addLocal(sumAngular);
		//it's not correct!

		// the mouvement from the angular accel is taken care the same way as
		// the rot from JointRotation
		// => on pointRotation
		// easy!

		// use this in jointPose!!
		// if(sumAngular.lengthSquared()>0){
		// //use an overly-simplified model of moment of inertia.
		// // moment of intertia: boule = mr²*2/5 tige(rot extrem): mL²/3
		// (4mr²/3)
		// // pavé (sur axe x): m*(y²+z²)/12
		// // our own : m*r²/2 = m*L²/8
		// sumAngular.divideLocal((float)(f.roundBBRayon * f.roundBBRayon *
		// f.mass / 2));
		//
		// //now, with this accel, check the normal force at angular point.
		//
		// //the normal direction is OP X ROT
		// Vector3f dirFromAngular = sumAngular.normalize().crossLocal(normalN);
		//
		// }
	}

	@Override
	public void addCollisionPoint(Vector3f pointCollision, int idx, Forme fOpposite, int idxOpposite) {
		assert !pointCollision.equals(pointWRotation) : "Error, add on the existing rotation point";
		System.out.println("joint ponctuel:@" + pointWRotation + " : add " + pointCollision);
		f.joint = new JointPose(f);
		//TODO
		f.joint.addCollisionPoint(this.pointWRotation, this.idxPoint, this.fOpposite, this.idxOpposite);
		f.joint.addCollisionPoint(pointCollision, idx, fOpposite, idxOpposite);

	}

	@Override
	public Collection<Integer> getIdx() {
		return Arrays.asList(idxPoint);
	}

	@Override
	public int degreeOfLiberty() {
		return 2;
	}

	@Override
	public void removeCollisionPoint(Vector3f pointCollision, int idx) {
		System.out.println(" JR "+f.name+" removeCollisionPoint "+idx+"@"+pointCollision);
		f.joint = new JointFreeFlight(f);
	}

}
