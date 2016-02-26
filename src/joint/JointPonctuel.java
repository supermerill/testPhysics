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

		System.out.println("JPc new Joint ponctuel has: " + f + " : " + pointWRotation + ".distance("
				+ f.transfoMatrix.mult(pointLRotation) + ") ="
				+ pointWRotation.distance(f.transfoMatrix.mult(pointLRotation)) + " > 0.0001");
	}

	@Override
	public void updatePosition(long instant, long dt) {
		super.updatePosition(instant, dt);
		// recaler le solide
		System.out.println("BeforeRecalage : " + pointWRotation + ".distance("
				+ f.transfoMatrix.mult(f.points.get(idxPoint)) + ") ="
				+ pointWRotation.distance(f.transfoMatrix.mult(new Vector3f(f.points.get(idxPoint)))));
		System.out.println("UpdatePos : recalage from " + f + "@" + f.position + " of "
				+ pointWRotation.subtract(f.transfoMatrix.mult(new Vector3f(f.points.get(idxPoint)))));
		f.position.addLocal(pointWRotation.subtract(f.transfoMatrix.mult(new Vector3f(f.points.get(idxPoint)))));
		f.transfoMatrix.setTranslation(f.position.toVec3f());
		// check if it has moved (more than 0.1mm)
		if (freeFlight || pointWRotation.distance(f.transfoMatrix.mult(pointLRotation)) > 0.0001f) {
			f.joint = new JointFreeFlight(f);
		}
		System.out.println("AfterRecalage : " + pointWRotation + ".distance("
				+ f.transfoMatrix.mult(f.points.get(idxPoint)) + ") ="
				+ pointWRotation.distance(f.transfoMatrix.mult(new Vector3f(f.points.get(idxPoint)))));

	}

	@Override
	public void updateForce(long instant, long dt) {
		System.out.println("JPc "+f.forces.size());
		if(f.forces.size()==0){
			//most common case, really, as you need a point of contact to have a force, in general.
			updateGravityOnly(instant, dt);
		}else{
			// used when your point of contact add some force on you.
			// & when magical forces begin to interact with you
			updateForceTODO(instant, dt);
		}
	}

	// from jointPose 26/02/2015
	// not working, i stop mid-thign
	public void updateForceTODO(long instant, long dt) {
		if (f.landed)
			return;

		// what i want to do:
		// 1) compute rotation from angular on rotationPoint (if in the way,
		// else 0)
		// 2) compute roation from linear on rotationPoint (if in the way, else
		// 0)
		// 3) add the previous angular force
		// pb: angualr to escape and linear to push on rp. => what if linearWin?
		// => it oscilate with FF?
		// 4) now aply, angular@O angular@RP, linear

		Vector3f fPos = f.position.toVec3f();
		// Vector3f angularAtO = new Vector3f(); == sumAngular
		// Vector3f angularAtRP = new Vector3f(); == newSumAngular
		// Vector3f linear = new Vector3f(); == sumLinear

		System.out.println("JPc updateForce! " + f.forces.size());
		Vector3f sumLinear = new Vector3f(0, 0, 0);
		Vector3f newSumLinear = new Vector3f(0, 0, 0);
		// add gravity
		if (!f.landed) {
			Vector3f vectGrav = f.positionGravite.subtract(fPos);
			float dist = vectGrav.length();
			sumLinear.addLocal(vectGrav.normalizeLocal().multLocal((float) (f.constanteGraviteMasse / (dist * dist))));
		}
		System.out.println("JPc linear gravity : " + sumLinear);

		// step 1.A) : compute all force on the body, decompose them into linear
		// and angular
		Vector3f sumAngular = new Vector3f(0, 0, 0);
		Vector3f newSumAngular = new Vector3f(0, 0, 0);
		Vector3f vectDir = new Vector3f();
		float sumPercent = 0;
		ArrayList<Float> percentAngular = new ArrayList<>();
		// angularforce: the angular moment contribution at this point
		ArrayList<Vector3f> angularForce = new ArrayList<>();
		// angularlinearforce: the linear force (in N) at that point that
		// convert 100% to angular motion
		ArrayList<Vector3f> angularLinearForce = new ArrayList<>();
		for (int i = 0; i < f.forces.size(); i++) {
			percentAngular.add(0f);
			angularLinearForce.add(new Vector3f());
			angularForce.add(new Vector3f());
			vectDir.set(fPos).subtractLocal(f.pointApplicationForce.get(i)).normalizeLocal();
			System.out.println("JPc force : " + f.forces.get(i));
			if (vectDir.lengthSquared() == 0) {
				// System.out.println("onlylinear : "+f.forces.get(i));
				sumLinear.addLocal(f.forces.get(i));
			} else {
				float length = f.forces.get(i).length();
				float dotAbs = Math.abs(vectDir.dot(f.forces.get(i)));
				sumLinear.addLocal(f.forces.get(i).mult(dotAbs / length));
				System.out.println("JPc dot : " + dotAbs + ", length=" + length);
				System.out.println("JPc linear : " + f.forces.get(i).mult(dotAbs / length));
				if (dotAbs < length) { // +epsilon?
					Vector3f angularVect = f.forces.get(i).cross(vectDir);
					Vector3f angularLinearVect = f.forces.get(i).mult((length - dotAbs) / length);
					angularVect.multLocal((length - dotAbs) / length);
					System.out.println("JPc angular : " + angularVect + " " + f.forces.get(i) + " cross " + vectDir
							+ " mult " + ((length - dotAbs) / length));

					// recompose angular into linear if combine
					// note: it should not occur on a free-flight object.
					float maxLength = Math.min(length, sumAngular.length());
					if (maxLength > 0) {
						float percent = angularVect.normalize().dot(sumAngular.normalize());
						// quantity of linear to re-add *2 if this angular force
						// isn't completely compensated
						float quantity = percent * (length - dotAbs) / length;
						// quantity of linear to re-add *2 if this angular force
						// is completely compensated
						if (percent * sumAngular.length() > quantity) {
							quantity = length;
						}
						if (percent < 0) {
							// System.out.println("addLinear : "+f.forces.get(i).mult(-2*percent*(length-dot)/length));
							// *2 because there are this force and the other
							// force which compensate me
							sumLinear.addLocal(f.forces.get(i).mult(-2 * quantity));

							// remove some from other angular forces
							float totPercentRemove = 0;
							for (int j = 0; j < i; j++) {
								totPercentRemove += angularVect.normalize().dot(angularForce.get(j).normalize());
							}
							for (int j = 0; j < i; j++) {
								float percentRemove = angularVect.normalize().dot(angularForce.get(j).normalize());
								percentRemove = percentRemove / totPercentRemove;
								float quantityRemove = quantity * percentRemove;
								percentAngular.set(j, percentAngular.get(j) - quantityRemove);
								angularLinearForce.set(j, angularLinearForce.get(j).mult(1 - percentRemove));
								angularForce.set(j, angularForce.get(j).mult(1 - percentRemove));
							}
							sumPercent -= quantity;
						} else {
							percentAngular.set(i, quantity);
							sumPercent += quantity;
							sumAngular.addLocal(angularVect);
							angularLinearForce.set(i, angularLinearVect);
							angularForce.set(i, angularVect);
						}
					} else {
						percentAngular.set(i, 1f);
						sumPercent += 1f;
						sumAngular.addLocal(angularVect);
						angularLinearForce.set(i, angularLinearVect);
						angularForce.set(i, angularVect);
					}
				}
			}
		}
		System.out.println("JPc linear intermediaire : " + sumLinear);
		System.out.println("JPc angular intermediaire : " + sumAngular);

		// step 1.B) : change the rotation point if it's in the way for the
		// angular force
		if (sumAngular.lengthSquared() > 0) {
			// Plane planRotation = new Plane(sumAngular, 0);
			Vector3f vect = fPos.subtract(pointWRotation);
			float checkOrientation = sumAngular.cross(vect).dot(f.normales.get(this.idxPoint));
			if (checkOrientation < 0) {
				// check dist
				// float val =
				// planRotation.getClosestPoint(vect).lengthSquared();
				// if(val > bestDist){
				// bestPointIdx = i;
				// bestDist = val;
				// bestPoint = vect;
				// }

				Vector3f pointRotation = pointWRotation;
				newSumLinear.set(sumLinear);

				// pour chaque point, on prend sa force en N (lineaire)
				// on prend le centre de gravité comme opposition
				// on pourra le re-transposer plus tard avec un nouveau centre
				// de rotation (ou le meme)
				for (int i = 0; i < f.forces.size(); i++) {
					if (percentAngular.get(i) > 0) {
						float FP = pointRotation.subtract(f.pointApplicationForce.get(i)).length();
						float PC = pointRotation.subtract(fPos).length();
						newSumAngular.addLocal(angularLinearForce.get(i).mult(FP / PC));
						System.out.println("JPc angularForce " + angularLinearForce.get(i) + " @"
								+ f.pointApplicationForce.get(i));
						System.out.println("JPc FP " + FP);
						System.out.println("JPc PC " + PC);
					}
				}
				// Vector3f linearAtRot =
				// newSumAngular.cross(pointRotation.subtract(fPos));
				// Vector3f linearAtRot =
				// pointRotation.subtract(fPos).cross(newSumAngular);
				Vector3f linearAtRot = newSumAngular;

				// add it to linear force ^^ (and let the simu to oscilate from
				// pose/freeflight)
				// TODOAFTER: creer un nouveau lien pour eviter l'oscillation?
				newSumLinear.addLocal(linearAtRot);
				// force != acceleration?
				// force lineaire: N (ou kg*m/s²)
				// force (moment) angulaire : N*m (ou kg*m²/s²)
				System.out.println("JPc linear UPPPPPP! " + linearAtRot);
				System.out.println("JPc pointRotation " + pointRotation);
				System.out.println("JPc newSumAngular " + newSumAngular);
				System.out.println("JPc pointRotation.subtract(fPos) " + pointRotation.subtract(fPos));

				sumAngular.set(0, 0, 0);

			} else {
				System.out.println("JPc point " + this.idxPoint + " @" + pointWRotation
						+ " is not in the right direction for angular pivot");

				// nothing stop it to rotate => rotate!
				sumAngular.divideLocal(f.getIntertiaMoment());
				f.aangulaire.addLocal(sumAngular);
				System.out.println("JPc linear rotate! " + sumAngular);

			}

		}

		Vector3f sumForces = sumLinear;
		Vector3f sumForcesN = sumForces.normalize();

		System.out.println("JPc linear intermediaire2 sumLinear: " + sumLinear);
		System.out.println("JPc linear intermediaire2 newSumLinear: " + newSumLinear);
		System.out.println("JPc linear intermediaire2 sumAngular: " + sumAngular);
		System.out.println("JPc linear intermediaire2 newSumAngular: " + newSumAngular);

		// // check if pointWRotation is in the way of the linear force
		// if(f.normales.get(this.idxPoint).dot(sumForcesN) > 0){
		// System.out.println("JPc : rotation occur");
		//
		// // normalized vector for RO
		// Vector3f vectPointN =
		// f.position.toVec3f().subtractLocal(pointWRotation);
		// vectPointN.normalizeLocal();
		// // seems to be the part of sumforceN that go into the other object
		// instead to add angular force.
		// Vector3f normal = vectPointN.mult(vectPointN.dot(sumLinear));
		// // seems to be the rotational vector from linear force if it push
		// against pointWRot ?
		// Vector3f angularFfromLinear =
		// vectPointN.cross(sumLinear.normalize()).mult(sumLinear.length() -
		// normal.length());
		//
		// System.out.println("JPc vectPointN : " + vectPointN);
		// System.out.println("JPc length/totalLength : " +
		// vectPointN.dot(sumLinear) + " / " + sumLinear.length());
		// System.out.println("JPc sumLinear : " + sumLinear + " (length=" +
		// sumLinear.length()+")");
		// System.out.println("JPc normale : " + normal + " (length=" +
		// normal.length());
		// System.out.println("JPc angularFfromLinear : " + angularFfromLinear +
		// " (length=" + angularFfromLinear.length()+")");
		//
		//
		// System.out.println("JPc point @"+this.pointLRotation+" is in the right direction for angular pivot");
		// //check dist edit: ????
		// Plane planRotation = new Plane(sumAngular, 0);
		// float val =
		// planRotation.getClosestPoint(fPos.subtract(this.pointWRotation)).lengthSquared();
		// if(val > 0){
		// //choke point!
		// //add linear to sum linear
		// //TODOAFTER: integrate the linear compo better than that! (to not
		// move the point of rotation)
		//
		// // // il y a le point d'application de la force F1 en F
		// // // il y a la force contre laquel on lutte (oupas?) appliqué en C
		// // // il y a le point pivot de rotation P
		// // // => il y a une force F2 en C de taille |F1|*FP/PC et de
		// direction -F1
		//
		// Vector3f newSumAngular = new Vector3f(0,0,0);
		// for(int i=0; i<f.forces.size(); i++){
		// if(percentAngular.get(i)>0){
		// float FP =
		// pointWRotation.subtract(f.pointApplicationForce.get(i)).length();
		// float PC = pointWRotation.subtract(fPos).length();
		// newSumAngular.addLocal(angularForce.get(i).mult(FP/PC));
		// }
		// }
		//
		// // ????
		// if (normal.length() > 0){
		// //it can't be freeflight
		// newSumAngular.addLocal(angularFfromLinear);
		// newSumAngular.divideLocal(f.getIntertiaMoment());
		// f.aangulaire.set(newSumAngular);
		// System.out.println("JPc LEVIER+FORCE");
		// }else{
		// //it can be free flight
		// // Vector3f linearAtRot =
		// sumAngular.cross(pointWRotation.subtract(fpos));
		// // Vector3f isFreeflight = linearAtRot.subtract(normal);
		// // if (isFreeflight.dot(linearAtRot.add(sumLinear)) > 0) {
		// if(linearAtRot.add(sumLinear).length() > 0){
		// System.out.println("JPc LEVIER-FORCE => FREE");
		// // freeFlight = true;
		// fOpposite.joint.removeCollisionPoint(pointWRotation, idxOpposite);
		// f.joint = new JointFreeFlight(f);
		// f.joint.updateForce(instant, dt);
		// //en principe, changement de centre de rotation
		// // =>le moment change
		// // => changement de la vitesse angulaire
		// //TODOAFTER
		// //
		// //freeFlight = true;
		// }else{
		// System.out.println("JPc LEVIER-FORCE => ROT");
		// //pareil, en principe le moment n'est pas bon...?
		// newSumAngular.addLocal(angularFfromLinear);
		// newSumAngular.divideLocal(f.getIntertiaMoment());
		// f.aangulaire.addLocal(sumAngular);
		// }
		// }
		//
		// }//ELSE?????? FIXME?
		// else{
		// System.err.println("JPc EROOR in JPC : code TODO FISME!!!!!");
		// }
		//
		// }else{
		// System.out.println(("JPc : going into freeflight"));
		// //TODO
		// }

	}

	// KC: need to copose the angular force & the linear force => change to do
	// that in the pose way
	// , ie by applied linear force
	public void updateForceKC(long instant, long dt) {
		if (f.landed)
			return;

		// for each force, decompose them into linear & angular
		System.out.println("JPc updateForce! " + f.forces.size());
		Vector3f sumLinear = new Vector3f(0, 0, 0);
		Vector3f sumAngular = new Vector3f(0, 0, 0);
		Vector3f fpos = f.position.toVec3f();
		Vector3f vectDir = new Vector3f();
		float sumPercent = 0;
		ArrayList<Float> percentAngular = new ArrayList<>();
		ArrayList<Vector3f> angularForce = new ArrayList<>();
		for (int i = 0; i < f.forces.size(); i++) {
			percentAngular.add(0f);
			angularForce.add(new Vector3f());
			vectDir.set(fpos).subtractLocal(f.pointApplicationForce.get(i)).normalizeLocal();
			if (vectDir.lengthSquared() == 0) {
				// System.out.println("onlylinear : "+f.forces.get(i));
				sumLinear.addLocal(f.forces.get(i));
			} else {
				float length = f.forces.get(i).length();
				float dot = vectDir.dot(f.forces.get(i));
				sumLinear.addLocal(f.forces.get(i).mult(dot / length));
				// System.out.println("linear : "+vectDir.mult(dot));
				if (dot < length) { // +epsilon?
					Vector3f angularVect = f.forces.get(i).cross(vectDir);
					angularVect.multLocal((length - dot) / length);
					// System.out.println("angular : "+angularVect+" "+f.forces.get(i)+" cross "+vectDir+" mult "+((length-dot)/length));

					// recompose angular into linear if combine
					// note: it should not occur on a free-flight object.
					float maxLength = Math.min(length, sumAngular.length());
					if (maxLength > 0) {
						float percent = angularVect.normalize().dot(sumAngular.normalize());
						// quantity of linear to re-add *2 if this angular force
						// isn't completely compensated
						float quantity = percent * (length - dot) / length;
						// quantity of linear to re-add *2 if this angular force
						// is completely compensated
						if (percent * sumAngular.length() > quantity) {
							quantity = length;
						}
						if (percent < 0) {
							// System.out.println("addLinear : "+f.forces.get(i).mult(-2*percent*(length-dot)/length));
							// *2 because there are this force and the other
							// force which compensate me
							sumLinear.addLocal(f.forces.get(i).mult(-2 * quantity));

							// remove some from other angular forces
							float totPercentRemove = 0;
							for (int j = 0; j < i; j++) {
								totPercentRemove += angularVect.normalize().dot(angularForce.get(j).normalize());
							}
							for (int j = 0; j < i; j++) {
								float percentRemove = angularVect.normalize().dot(angularForce.get(j).normalize());
								percentRemove = percentRemove / totPercentRemove;
								float quantityRemove = quantity * percentRemove;
								percentAngular.set(j, percentAngular.get(j) - quantityRemove);
								angularForce.set(j, angularForce.get(j).mult(1 - percentRemove));
							}
							sumPercent -= quantity;
						} else {
							percentAngular.set(i, quantity);
							sumPercent += quantity;
							sumAngular.addLocal(angularVect);
							angularForce.set(i, angularVect);
						}
					} else {
						percentAngular.set(i, 1f);
						sumPercent += 1f;
						sumAngular.addLocal(angularVect);
						angularForce.set(i, angularVect);
					}
				}
			}
		}
		System.out.println("JPc linear intermediaire : " + sumLinear);
		System.out.println("JPc angular intermediaire : " + sumAngular);

		// now create the resultante on the contact point

		// create the normal force (via a dot)
		// f.acceleration.set(f.positionGravite).subtract(fpos);
		// sumLinear.divideLocal((float)f.mass);
		// f.acceleration.addLocal(sumLinear);

		// add gravity to sumlinear
		System.out.println("JPc sumLinear, before gravity : " + sumLinear + " (length=" + sumLinear.length() + ")");

		Vector3f gravity = f.positionGravite.subtract(fpos);
		float distGravity = gravity.length();
		gravity.normalizeLocal().multLocal((float) (f.constanteGraviteMasse / (distGravity * distGravity * f.mass)));
		// sumLinear.addLocal(f.positionGravite.subtract(fpos).normalize().mult((float)(f.mass
		// * f.constanteGraviteMasse)));
		sumLinear.addLocal(gravity);
		System.out.println("JPc Gravity = " + gravity);

		// normalized vector for RO
		Vector3f vectPointN = f.position.toVec3f().subtractLocal(pointWRotation);
		vectPointN.normalizeLocal();
		// seems to be the part of sumforceN that go into the other object
		// instead to add angular force.
		Vector3f normal = vectPointN.mult(vectPointN.dot(sumLinear));
		// seems to be the rotational vector from linear force if it push
		// against pointWRot ?
		Vector3f angularFfromLinear = vectPointN.cross(sumLinear.normalize())
				.mult(sumLinear.length() - normal.length());

		System.out.println("JPc vectPointN : " + vectPointN);
		System.out.println("JPc length/totalLength : " + vectPointN.dot(sumLinear) + " / " + sumLinear.length());
		System.out.println("JPc sumLinear : " + sumLinear + " (length=" + sumLinear.length() + ")");
		System.out.println("JPc normale : " + normal + " (length=" + normal.length());
		System.out.println("JPc angularFfromLinear : " + angularFfromLinear + " (length=" + angularFfromLinear.length()
				+ ")");

		if (normal.length() < 0) {
			// it's reversed!
			// freeFlight = true;

		} else {

			// create rotationVector
			// / ???
			// Vector3f rotVector =
			// vectPointN.cross(sumLinear.normalize()).mult(sumLinear.length() -
			// normal.length());
			// // rotVector.divideLocal(f.getIntertiaMoment());
			// angularFfromLinear.set(rotVector);
			// f.aangulaire.addLocal(rotVector);
			// System.out.println("sum : " + sumLinear);
			// System.out.println("cross : " +
			// normalN.cross(sumLinear.normalize()));
			// System.out.println("crossN : "
			// + normalN.cross(sumLinear.normalize()).mult(sumLinear.length() -
			// normal.length()));
			// System.out.println("rotVector : " + rotVector);
			// System.out.println("f.aangulaire : " + f.aangulaire);

		}

		// TODO FIXME linearAtRot & sumLinear -> need to 'add' them where?

		// create resultante on contact point for the angular force
		Vector3f linearAtRot = sumAngular.cross(pointWRotation.subtract(fpos));
		System.out.println("JPc sumAngular=" + sumAngular + " X pointWRotation.subtract(fpos)="
				+ pointWRotation.subtract(fpos) + " ==> linearAtRot=" + linearAtRot);
		// Vector3f vect = fpos.subtract(this.pointWRotation);
		float checkOrientation = linearAtRot.dot(f.normales.get(this.idxPoint));
		System.out.println("JPc checkOrientation=" + checkOrientation);
		Plane planRotation = new Plane(sumAngular, 0);
		if (checkOrientation >= 0) {
			System.out.println("JPc point @" + this.pointLRotation + " is in the right direction for angular pivot");
			// check dist edit: ????
			float val = planRotation.getClosestPoint(fpos.subtract(this.pointWRotation)).lengthSquared();
			if (val > 0) {
				// choke point!
				// add linear to sum linear
				// TODOAFTER: integrate the linear compo better than that! (to
				// not move the point of rotation)

				// // il y a le point d'application de la force F1 en F
				// // il y a la force contre laquel on lutte (oupas?) appliqué
				// en C
				// // il y a le point pivot de rotation P
				// // => il y a une force F2 en C de taille |F1|*FP/PC et de
				// direction -F1

				Vector3f newSumAngular = new Vector3f(0, 0, 0);
				for (int i = 0; i < f.forces.size(); i++) {
					if (percentAngular.get(i) > 0) {
						float FP = pointWRotation.subtract(f.pointApplicationForce.get(i)).length();
						float PC = pointWRotation.subtract(fpos).length();
						newSumAngular.addLocal(angularForce.get(i).mult(FP / PC));
					}
				}

				// ????
				if (normal.length() > 0) {
					// it can't be freeflight
					newSumAngular.addLocal(angularFfromLinear);
					newSumAngular.divideLocal(f.getIntertiaMoment());
					f.aangulaire.set(newSumAngular);
					System.out.println("JPc LEVIER+FORCE");
				} else {
					// it can be free flight
					// Vector3f linearAtRot =
					// sumAngular.cross(pointWRotation.subtract(fpos));
					// Vector3f isFreeflight = linearAtRot.subtract(normal);
					// if (isFreeflight.dot(linearAtRot.add(sumLinear)) > 0) {
					if (linearAtRot.add(sumLinear).length() > 0) {
						System.out.println("JPc LEVIER-FORCE => FREE");
						// freeFlight = true;
						fOpposite.joint.removeCollisionPoint(pointWRotation, idxOpposite);
						f.joint = new JointFreeFlight(f);
						f.joint.updateForce(instant, dt);
						// en principe, changement de centre de rotation
						// =>le moment change
						// => changement de la vitesse angulaire
						// TODOAFTER
						//
						// freeFlight = true;
					} else {
						System.out.println("JPc LEVIER-FORCE => ROT");
						// pareil, en principe le moment n'est pas bon...?
						newSumAngular.addLocal(angularFfromLinear);
						newSumAngular.divideLocal(f.getIntertiaMoment());
						f.aangulaire.addLocal(sumAngular);
					}
				}

			}// ELSE?????? FIXME?
		} else {
			System.out
					.println("JPc point @" + this.pointLRotation + " is not in the right direction for angular pivot");
			// => ajouter en tant que force de rotation "free flight"

			// TODO: this rotation is not in pointWRotation !!
			// sumAngular.divideLocal(f.getIntertiaMoment());
			// f.aangulaire.set(sumAngular);

			// check if it's free flight or not
			// Vector3f linearAtRot =
			// sumAngular.cross(pointWRotation.subtract(fpos));
			System.out.println("sumAngular=" + sumAngular + ", pointWRotation.subtract(fpos)="
					+ pointWRotation.subtract(fpos));
			// Vector3f isFreeflight = linearAtRot.subtract(normal); /// ????
			// System.out.println(isFreeflight+" dot "+linearAtRot+".add("+sumLinear+") = "+isFreeflight.dot(linearAtRot.add(sumLinear))+
			// " >? 0");
			System.out.println("linearAtRot=" + linearAtRot + ", sumLinear=" + sumLinear);
			if (linearAtRot.add(sumLinear).length() > 0) {
				System.out.println("JPc -LEVIER-+FORCE => FREE");
				// freeFlight = true;
				fOpposite.joint.removeCollisionPoint(pointWRotation, idxOpposite);
				f.joint = new JointFreeFlight(f);
				f.joint.updateForce(instant, dt);
				// en principe, changement de centre de rotation
				// =>le moment change
				// => changement de la vitesse angulaire
				// TODOAFTER
				//
				// freeFlight = true;
			} else {
				System.out.println("JPc -LEVIER-+FORCE => ROT");
				// pareil, en principe le moment n'est pas bon...
				sumAngular.addLocal(angularFfromLinear);
				sumAngular.divideLocal(f.getIntertiaMoment());
				f.aangulaire.addLocal(sumAngular);
			}

		}
		System.out.println("JPc f.aangulaire=" + f.aangulaire);

		// f.aangulaire.set(0, 0, 0);
		// // compute sum of force
		// Vector3f sumForce = new Vector3f(0, 0, 0);
		// for (Vector3f force : f.forces) {
		// sumForce.addLocal(force);
		// }

		// Angular force: just transpose them to angular acceleration
		// Vector3f sumAngular = new Vector3f(0, 0, 0);
		// for (Vector3f force : f.angularForces) {
		// sumAngular.addLocal(force);
		// }
		// sumAngular.divideLocal(f.getIntertiaMoment());
		// f.aangulaire.addLocal(sumAngular);
		// it's not correct!

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

	public void updateGravityOnly(long instant, long dt) {
		if (f.landed)
			return;

		f.aangulaire.set(0, 0, 0);
		

		Vector3f gravity = f.positionGravite.subtract(f.position.toVec3f());
		float distGravity = gravity.length();
		gravity.normalizeLocal().multLocal((float) (f.constanteGraviteMasse / (distGravity * distGravity * f.mass)));
		System.out.println("JPc Gravity = " + gravity);

		// create the normal force (via a dot)
		Vector3f normalN = f.position.toVec3f().subtractLocal(pointWRotation);
		normalN.normalizeLocal();
		Vector3f normal = normalN.mult(normalN.dot(gravity));
		System.out.println("JPc normaleN : " + normalN);
		System.out.println("JPc length/totalLength : " + normalN.dot(gravity) + " / " + gravity.length());
		System.out.println("JPc normale : " + normal + " (length=" + normal.length());
		if (normal.length() < 0) {
			// it's reversed!
			freeFlight = true;
		} else {

			// create rotationVector
			// / ???
			Vector3f rotVector = normalN.cross(gravity.normalize()).mult(gravity.length() - normal.length());
			rotVector.divideLocal(f.getIntertiaMoment());
			f.aangulaire.addLocal(rotVector);
			System.out.println("JPc sum : " + gravity);
			System.out.println("JPc cross : " + normalN.cross(gravity.normalize()));
			System.out.println("JPc crossN : "
					+ normalN.cross(gravity.normalize()).mult(gravity.length() - normal.length()));
			System.out.println("JPc rotVector : " + rotVector);
			System.out.println("JPc f.aangulaire : " + f.aangulaire);

		}

		// Angular force: none

	}

	@Override
	public void addCollisionPoint(Vector3f pointCollision, int idx, Forme fOpposite, int idxOpposite) {
		assert !pointCollision.equals(pointWRotation) : "Error, add on the existing rotation point";
		System.out.println("joint ponctuel:@" + pointWRotation + " : add " + pointCollision);
		f.joint = new JointPose(f);
		// TODO
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
		System.out.println(" JR " + f.name + " removeCollisionPoint " + idx + "@" + pointCollision);
		f.joint = new JointFreeFlight(f);
	}

}
