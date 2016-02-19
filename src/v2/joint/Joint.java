package v2.joint;

import java.util.ArrayList;
import java.util.Collection;

import v2.Forme;

import jme3Double.PlaneD;
import jme3Double.Vector3d;

public abstract class Joint {
	
	public Forme f;
	
	Joint(Forme f){
		this.f = f;
	}
	
	//update force & update pos
//	public void update(long instant, long dt){
//		System.out.println("update joint force for forme "+f+"@"+f.position);
//		updateForce(instant, dt);
//		System.out.println("update joint pos for forme "+f+"@"+f.position);
//		updatePosition(instant, dt);
//		System.out.println("update joint ended for forme "+f+"@"+f.position);
//	}

	//update rot&linear speed&position
	public abstract void updatePosition(long instant, long dt);
	
	// cehck all forces, tranfert them to others formes, change angular&linear accel
	public abstract void updateForce(long instant, long dt);

	// force = masse * acceleration
	// energiecinétique = 0.5*mv² 
	//TODO? energy in kg*m/s²
	public void receiveEnergy(float force, Vector3d position, Vector3d direction){
		
	}

	public abstract void addCollisionPoint(Vector3d pointCollision, int idx, Forme fOpposite, int idxOpposite);
	public abstract void removeCollisionPoint(Vector3d pointCollision, int idx);

	public Collection<Integer> getIdx() {
		return new ArrayList<>(0);
	}
	
	public int degreeOfLiberty(){
		return 3;
	}

	public abstract void gotoCollision(int pointIdx, Vector3d pointObj);

	public abstract void gotoCollision(int pointIdx, PlaneD planeObj);

	public abstract void gotoCollision(Vector3d localTA, Vector3d localTB, Vector3d localTC, Vector3d worldObj);

	
}
