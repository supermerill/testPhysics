package v2.joint;

import v2.Forme;
import jme3Double.PlaneD;
import jme3Double.Vector3d;

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
	public void addCollisionPoint(Vector3d pointCollision, int idx, Forme fOpposite, int idxOpposite) {
		// TODO?
	}

	@Override
	public void removeCollisionPoint(Vector3d pointCollision, int idx) {
	}

	public int degreeOfLiberty() {
		return -1;
	}

	@Override
	public void gotoCollision(int pointIdx, Vector3d pointObj) {
		// NOP
	}

	@Override
	public void gotoCollision(int pointIdx, PlaneD planeObj) {
		// NOP
	}

	@Override
	public void gotoCollision(Vector3d localTA, Vector3d localTB, Vector3d localTC, Vector3d worldObj){
		// NOP
	}

}
