package v2.collision;

import v2.Forme;


public class CollisionPrediction {
	
	public long moment;
	public Precision precisionMoment;
	public double maxMomentError; // if precisionMoment == PRECISE
	public Forme forme1;
	public Forme forme2;
	
	static enum Precision{
		INITIAL, // using bbbox
		REFINED, // using some other tech like ray-tri & tri-tri dist
		INTEGRATED, // integrated & mesh-mesh collision detection
		CANTFIND // when, well, we can't find the collision, so it may not exist after all.
	}
	
}
