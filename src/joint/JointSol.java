package joint;

import com.jme3.math.Plane;
import com.jme3.math.Vector3f;

import old.Forme;

public class JointSol extends Joint {

	JointSol(Forme f) {
		super(f);
	}

	@Override
	public void updatePosition(long instant, long dt) {

	}

	@Override
	public void updateForce(long instant, long dt) {

	}

	@Override
	public void addCollisionPoint(Vector3f pointCollision, int idx, Forme fOpposite, int idxOpposite) {
		// TODO?
	}

	@Override
	public void removeCollisionPoint(Vector3f pointCollision, int idx) {
	}

	public int degreeOfLiberty() {
		return -1;
	}

	@Override
	public void gotoCollision(int pointIdx, Vector3f pointObj) {
		// NOP
	}

	@Override
	public void gotoCollision(int pointIdx, Plane planeObj) {
		// NOP
	}

	@Override
	public void gotoCollision(Vector3f localTA, Vector3f localTB, Vector3f localTC, Vector3f worldObj){
		// NOP
	}

}
