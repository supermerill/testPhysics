package joint;

import com.jme3.math.Vector3f;

import old.Forme;

public class JointSol extends Joint{

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
	public void addCollisionPoint(Vector3f pointCollision, int idx) {
		//TODO?
	}

	public int degreeOfLiberty(){
		return -1;
	}
	

}
