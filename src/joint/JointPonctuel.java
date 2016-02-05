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
		

		System.out.println("JP new Joint ponctuel has: "+f+" : " + pointWRotation + ".distance("
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
		if (freeFlight) {
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

		f.aangulaire.set(0, 0, 0);
		// compute sum of force
		Vector3f sumForce = new Vector3f(0, 0, 0);
		for (Vector3f force : f.forces) {
			sumForce.addLocal(force);
		}

		// create the normal force (via a dot)
		Vector3f normalN = f.position.toVec3f().subtractLocal(pointWRotation);
		normalN.normalizeLocal();
		Vector3f normal = normalN.mult(normalN.dot(sumForce));
		System.out.println("normaleN : " + normalN);
		System.out.println("length/totalLength : " + normalN.dot(sumForce) + " / " + sumForce.length());
		System.out.println("normale : " + normal + " (length=" + normal.length());
		if (normal.length() < 0) {
			// it's reversed!
			freeFlight = true;
		} else {

			// create rotationVector
			/// ???
			Vector3f rotVector = normalN.cross(sumForce.normalize()).mult(sumForce.length() - normal.length());
			rotVector.divideLocal(f.getIntertiaMoment());
			f.aangulaire.addLocal(rotVector);
			System.out.println("sum : " + sumForce);
			System.out.println("cross : " + normalN.cross(sumForce.normalize()));
			System.out.println("crossN : "
					+ normalN.cross(sumForce.normalize()).mult(sumForce.length() - normal.length()));
			System.out.println("rotVector : " + rotVector);
			System.out.println("f.aangulaire : " + f.aangulaire);

		}

		// Angular force: just transpose them to angular acceleration
		Vector3f sumAngular = new Vector3f(0, 0, 0);
		for (Vector3f force : f.angularForces) {
			sumAngular.addLocal(force);
		}
		sumAngular.divideLocal(f.getIntertiaMoment());
		f.aangulaire.addLocal(sumAngular);

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
		f.joint = new JointFreeFlight(f);
	}

}
