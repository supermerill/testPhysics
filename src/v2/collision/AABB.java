package v2.collision;

import jme3Double.Vector3d;

public class AABB {

	public double minX, maxX;
	public double minY, maxY;
	public double minZ, maxZ;

	public void set(Vector3d center, double rayon, Vector3d speed, long time){
		minX = center.x - rayon;
		maxX = center.x + rayon;
		minY = center.y - rayon;
		maxY = center.y + rayon;
		minZ = center.z - rayon;
		maxZ = center.z + rayon;

		if(speed.x<0) minX += speed.x*time; else maxX += speed.x*time;
		if(speed.y<0) minY += speed.y*time; else maxY += speed.y*time;
		if(speed.z<0) minZ += speed.z*time; else maxZ += speed.z*time;
		
	}

	public void set(Vector3d center, double rayon, Vector3d displace){
		minX = center.x - rayon;
		maxX = center.x + rayon;
		minY = center.y - rayon;
		maxY = center.y + rayon;
		minZ = center.z - rayon;
		maxZ = center.z + rayon;

		if(displace.x<0) minX += displace.x; else maxX += displace.x;
		if(displace.y<0) minY += displace.y; else maxY += displace.y;
		if(displace.z<0) minZ += displace.z; else maxZ += displace.z;
		
	}
	
	//from internet, to check irl
	public boolean collide(AABB other){
		AABB a  = this;
		AABB b = other;
		
		//check if other inside this
		boolean ret =
				a.minX >= b.minX && a.maxX <= b.maxX &&
		        a.minY >= b.minY && a.maxY <= b.maxY &&
		        a.minZ >= b.minZ && a.maxZ <= b.maxZ;

		//check if this inside other
		a  = other;
		b = this;
		ret = ret ||
				a.minX >= b.minX && a.maxX <= b.maxX &&
		        a.minY >= b.minY && a.maxY <= b.maxY &&
		        a.minZ >= b.minZ && a.maxZ <= b.maxZ;

		//check if this collide other
		return ret ||
				(a.minX <= b.maxX && a.maxX >= b.minX) &&
		         (a.minY <= b.maxY && a.maxY >= b.minY) &&
		         (a.minZ <= b.maxZ && a.maxZ >= b.minZ);
		
	}
	
	
}
