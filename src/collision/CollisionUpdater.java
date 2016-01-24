package collision;

import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Set;

import old.Forme;

public class CollisionUpdater {
	
	// prediction of collision, list to re-check (because of rotation, for incoming other objects, near the predicted object)
	Map<Long, CollisionPrediction> toCheck = new HashMap<>();
	
	// this forme has suffer external force (en plus de la gravité) => recheck prediction
	Set<Forme> toUpdate = new HashSet<>();
	

}
