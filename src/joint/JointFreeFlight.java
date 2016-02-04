package joint;

import jme3Double.Vector3d;
import old.Forme;

import com.jme3.math.Plane;
import com.jme3.math.Quaternion;
import com.jme3.math.Vector3f;

public class JointFreeFlight extends Joint {

	public JointFreeFlight(Forme f) {
		super(f);
	}

	@Override
	public void updatePosition(long instant, long dt) {
		//all of this is done in collision update
		// move linear
//		f.calculF.set(f.vitesse).multLocal(dt);
//		f.position.addLocal(f.calculF);
//		f.calculF.set(f.lastAccel).multLocal(dt * dt);
//		f.position.addLocal(f.calculF);
//
//		// move angular
//		// keep quaternion? => it reduce rounding error
//		f.pangulaire.fromRotationMatrix(f.transfoMatrix.toRotationMatrix());
//		// float angle = f.vangulaire.length();
//		// if (angle > 0.0) // the formulas from the link
//		// {
//		// f.pangulaire.addLocal(new Quaternion(
//		// f.vangulaire.x * FastMath.sin(angle / 2.0f) / angle,
//		// f.vangulaire.y * FastMath.sin(angle / 2.0f) / angle,
//		// f.vangulaire.z * FastMath.sin(angle / 2.0f) / angle,
//		// FastMath.cos(angle / 2.0f)));
//		// }
//		Quaternion quaterAdd = new Quaternion().fromAngleAxis(f.vangulaire.length() * dt, f.vangulaire);
//		f.pangulaire.multLocal(quaterAdd);
//		f.pangulaire.normalizeLocal();
//
//		f.transfoMatrix.setTransform(f.position.toVec3fLocal(f.calculF), Vector3f.UNIT_XYZ,
//				f.pangulaire.toRotationMatrix());
//
//		// update linear speed
//		f.calculF.set(f.lastAccel).multLocal(dt * 0.5f);
//		f.vitesse.addLocal(f.calculF);
//		f.calculF.set(f.acceleration).multLocal(dt * 0.5f);
//		f.vitesse.addLocal(f.calculF);
//
//		// update angular speed
//		f.calculF.set(f.lastAangulaire).multLocal(dt * 0.5f);
//		f.vangulaire.addLocal(f.calculF);
//		f.calculF.set(f.aangulaire).multLocal(dt * 0.5f);
//		f.vangulaire.addLocal(f.calculF);
//
//		// update accel
//		f.lastAccel.set(f.acceleration);
//		f.lastAangulaire.set(f.aangulaire);
	}

	@Override
	public void updateForce(long instant, long dt) {
		//force -> accel
		//https://pixelastic.github.io/pokemonorbigdata/


		f.acceleration.set(0,0,0);
		for (Vector3f force : f.forces) {
			f.acceleration.addLocal(force);
		}
		System.out.println("sumForces : " + f.acceleration);
		
		//decomposition en linéaire et angulaire
		//TODO: need position of forces to do that?
		// (or not as thay can be decomposed by linear force and angular force)
		
		//Angular force: just transpose them to angular acceleration
		f.aangulaire.set(0,0,0);
		for (Vector3f force : f.angularForces) {
			f.aangulaire.addLocal(force);
		}
		System.out.println("f.aangulaire="+f.aangulaire);
		f.aangulaire.divideLocal(f.getIntertiaMoment());
		
		//for each force : OF dot F1F2 => lineaire
		//the remaining is rot with OF/2 as center
		// => decompose remaining rotation force as lineaire and angular
		
		f.acceleration.divideLocal((float)f.mass);
		System.out.println("accel : "+ f.acceleration);
		
	}

	@Override
	public void addCollisionPoint(Vector3f pointCollision, int idx, Forme fOpposite, int idxOpposite) {
		f.joint = new JointPonctuel(f, pointCollision, idx, fOpposite, idxOpposite);
	}

	@Override
	public void gotoCollision(int pointIdx, Vector3f pointObj) {
		//check diff
		Vector3f worldPos = new Vector3f();
		f.transfoMatrix.mult(f.points.get(pointIdx), worldPos);
		System.out.println("  worldPos="+worldPos+" to pointObj="+pointObj);
		f.position.addLocal(new Vector3d(pointObj.subtract(worldPos)));
		f.transfoMatrix.setTranslation(f.position.toVec3f());
	}

	@Override
	public void removeCollisionPoint(Vector3f pointCollision, int idx) {
	}

	@Override
	public void gotoCollision(int pointIdx, Plane planeObj) {
		Vector3f worldPos = new Vector3f();
		f.transfoMatrix.mult(f.points.get(pointIdx), worldPos);
		Vector3f pointObj = planeObj.getClosestPoint(worldPos);
		System.out.println("  worldPos="+worldPos+" to pointObj="+pointObj);
		f.position.addLocal(new Vector3d(pointObj.subtract(worldPos)));
		f.transfoMatrix.setTranslation(f.position.toVec3f());
	}

	@Override
	public void gotoCollision(Vector3f localTA, Vector3f localTB, Vector3f localTC, Vector3f worldObj) {
		Plane p = new Plane();
		Vector3f tempA = f.transfoMatrix.mult(localTA, new Vector3f());
		Vector3f tempB = f.transfoMatrix.mult(localTB, new Vector3f());
		Vector3f tempC = f.transfoMatrix.mult(localTC, new Vector3f());
		p.setPlanePoints(tempA, tempB, tempC);
		Vector3f worldPos =p.getClosestPoint(worldObj);
		System.out.println("  worldPos="+worldPos+" to pointObj="+worldObj);
		f.position.addLocal(new Vector3d(worldObj.subtract(worldPos)));
		f.transfoMatrix.setTranslation(f.position.toVec3f());
		
		//TODO: faire un peu de rotation en meme temps!
	}
}
