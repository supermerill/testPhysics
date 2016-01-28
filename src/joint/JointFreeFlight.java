package joint;

import com.jme3.math.Quaternion;
import com.jme3.math.Vector3f;

public class JointFreeFlight extends Joint {

	@Override
	public void updatePosition(long instant, long dt) {
		// move linear
		f.calculF.set(f.vitesse).multLocal(dt);
		f.position.addLocal(f.calculF);
		f.calculF.set(f.lastAccel).multLocal(dt * dt);
		f.position.addLocal(f.calculF);

		// move angular
		// keep quaternion? => it reduce rounding error
		f.pangulaire.fromRotationMatrix(f.transfoMatrix.toRotationMatrix());
		// float angle = f.vangulaire.length();
		// if (angle > 0.0) // the formulas from the link
		// {
		// f.pangulaire.addLocal(new Quaternion(
		// f.vangulaire.x * FastMath.sin(angle / 2.0f) / angle,
		// f.vangulaire.y * FastMath.sin(angle / 2.0f) / angle,
		// f.vangulaire.z * FastMath.sin(angle / 2.0f) / angle,
		// FastMath.cos(angle / 2.0f)));
		// }
		Quaternion quaterAdd = new Quaternion().fromAngleAxis(f.vangulaire.length() * dt, f.vangulaire);
		f.pangulaire.multLocal(quaterAdd);
		f.pangulaire.normalizeLocal();

		f.transfoMatrix.setTransform(f.position.toVec3fLocal(f.calculF), Vector3f.UNIT_XYZ,
				f.pangulaire.toRotationMatrix());

		// update linear speed
		f.calculF.set(f.lastAccel).multLocal(dt * 0.5f);
		f.vitesse.addLocal(f.calculF);
		f.calculF.set(f.acceleration).multLocal(dt * 0.5f);
		f.vitesse.addLocal(f.calculF);

		// update angular speed
		f.calculF.set(f.lastAangulaire).multLocal(dt * 0.5f);
		f.vangulaire.addLocal(f.calculF);
		f.calculF.set(f.aangulaire).multLocal(dt * 0.5f);
		f.vangulaire.addLocal(f.calculF);

		// update accel
		f.lastAccel.set(f.acceleration);
		f.lastAangulaire.set(f.aangulaire);
	}

	@Override
	public void updateForce(long instant, long dt) {
		//nothing to do.https://pixelastic.github.io/pokemonorbigdata/

	}

}
