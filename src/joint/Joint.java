package joint;

import java.util.ArrayList;
import java.util.Collection;

import com.jme3.math.Vector3f;

import old.Forme;

public abstract class Joint {
	
	public Forme f;
	
	Joint(Forme f){
		this.f = f;
	}
	
	//update force & update pos
	public void update(long instant, long dt){
		updateForce(instant, dt);
		updatePosition(instant, dt);
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
	
	public abstract void addCollisionPoint(Vector3f pointCollision, int idx);

	public Collection<Integer> getIdx() {
		return new ArrayList<>(0);
	}
	
}
