package joint;

import java.util.ArrayList;
import java.util.Collection;

import com.jme3.math.Plane;
import com.jme3.math.Vector3f;

import old.Forme;

public abstract class Joint {
	
	public Forme f;
	
	Joint(Forme f){
		this.f = f;
	}
	
	//update force & update pos
	public void update(long instant, long dt){
		System.out.println("update joint force for forme "+f+"@"+f.position);
		updateForce(instant, dt);
		System.out.println("update joint pos for forme "+f+"@"+f.position);
		updatePosition(instant, dt);
		System.out.println("update joint ended for forme "+f+"@"+f.position);
	}

	//update rot&linear speed&position
	public abstract void updatePosition(long instant, long dt);
	
	// cehck all forces, tranfert them to others formes, change angular&linear accel
	public abstract void updateForce(long instant, long dt);

	// force = masse * acceleration
	// energiecinétique = 0.5*mv² 
	//TODO? energy in kg*m/s²
	public void receiveEnergy(float force, Vector3f position, Vector3f direction){
		
	}

	public abstract void addCollisionPoint(Vector3f pointCollision, int idx, Forme fOpposite, int idxOpposite);
	public abstract void removeCollisionPoint(Vector3f pointCollision, int idx);

	public Collection<Integer> getIdx() {
		return new ArrayList<>(0);
	}
	
	public int degreeOfLiberty(){
		return 3;
	}

	public abstract void gotoCollision(int pointIdx, Vector3f pointObj);

	public abstract void gotoCollision(int pointIdx, Plane planeObj);

	public abstract void gotoCollision(Vector3f localTA, Vector3f localTB, Vector3f localTC, Vector3f worldObj);

	
}
