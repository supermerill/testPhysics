package joint;

import java.util.ArrayList;
import java.util.List;

import com.jme3.math.Plane;
import com.jme3.math.Vector3f;

import old.Forme;

public class ForceResolver {

	private List<Vector3f> optiCallV = new ArrayList<>(1);
	private List<Integer> optiCallI = new ArrayList<>(1);
	
	Forme f;

	Vector3f angularAtGravityCenter = new Vector3f();
	Vector3f linearAtGravityCenter = new Vector3f();
	Vector3f angularAtRotationPoint = new Vector3f();
	Vector3f rotationPoint;
	
	void computeLinearAtCenterOfMass(Vector3f point, int pointIdx){
		optiCallV.clear();
		optiCallV.add(point);
		optiCallI.clear();
		optiCallI.add(pointIdx);
		computeLinearAtCenterOfMass(optiCallV,optiCallI);
	}
	
	void computeLinearAtCenterOfMass(List<Vector3f> points, List<Integer> pointsIdx){
		Vector3f fPos = f.position.toVec3f();
		System.out.println("FR updateForce! "+f.forces.size());
		Vector3f sumLinear = new Vector3f(0,0,0);
		//add gravity
		if(!f.landed){
			Vector3f vectGrav = f.positionGravite.subtract(fPos);
			float dist = vectGrav.length();
			sumLinear.addLocal(vectGrav.normalizeLocal().multLocal((float)(f.constanteGraviteMasse/(dist*dist))));
		}
		System.out.println("FR linear gravity : "+sumLinear);
		Vector3f sumAngular = new Vector3f(0,0,0);
		Vector3f vectDir = new Vector3f();
		float sumPercent = 0;
		ArrayList<Float> percentAngular = new ArrayList<>();
		//angularforce: the angular moment contribution at this point
		ArrayList<Vector3f> angularForce = new ArrayList<>();
		//angularlinearforce: the linear force (in N) at that point that convert 100% to angular motion
		ArrayList<Vector3f> angularLinearForce = new ArrayList<>();
		for(int i=0; i<f.forces.size(); i++){
			percentAngular.add(0f);
			angularLinearForce.add(new Vector3f());
			angularForce.add(new Vector3f());
			vectDir.set(fPos).subtractLocal(f.pointApplicationForce.get(i)).normalizeLocal();
			System.out.println("FR force : "+f.forces.get(i));
			if(vectDir.lengthSquared() == 0){
//						System.out.println("onlylinear : "+f.forces.get(i));
				sumLinear.addLocal(f.forces.get(i));
			}else{
				float length = f.forces.get(i).length();
				float dotAbs = Math.abs(vectDir.dot(f.forces.get(i)));
				sumLinear.addLocal(f.forces.get(i).mult(dotAbs/length));
				System.out.println("FR dot : "+dotAbs+", length="+length);
				System.out.println("FR linear : "+f.forces.get(i).mult(dotAbs/length));
				if(dotAbs < length){ //+epsilon?
					Vector3f angularVect = f.forces.get(i).cross(vectDir);
					Vector3f angularLinearVect = f.forces.get(i).mult((length-dotAbs)/length);
					angularVect.multLocal((length-dotAbs)/length);
							System.out.println("angular : "+angularVect+" "+f.forces.get(i)+" cross "+vectDir+" mult "+((length-dotAbs)/length));
					
					//recompose angular into linear if combine
					// note: it should not occur on a free-flight object.
					float maxLength = Math.min(length, sumAngular.length());
					if(maxLength>0){
						//TODO:test more
						
						float percent = angularVect.normalize().dot(sumAngular.normalize());
						//quantity of linear to re-add *2 if this angular force isn't completely compensated
						float quantity = percent*(length-dotAbs)/length;
						//quantity of linear to re-add *2 if this angular force is completely compensated
						if(percent*sumAngular.length() > quantity){
							quantity = length;
						}
						if(percent < 0){
//									System.out.println("addLinear : "+f.forces.get(i).mult(-2*percent*(length-dot)/length));
							//*2 because there are this force and the other force which compensate me
							sumLinear.addLocal(f.forces.get(i).mult(-2*quantity));
							
							//remove some from other angular forces
							float totPercentRemove = 0;
							for(int j=0; j<i; j++){
								totPercentRemove += angularVect.normalize().dot(angularForce.get(j).normalize());
							}
							for(int j=0; j<i; j++){
								float percentRemove = angularVect.normalize().dot(angularForce.get(j).normalize());
								percentRemove = percentRemove / totPercentRemove;
								float quantityRemove = quantity * percentRemove;
								percentAngular.set(j,  percentAngular.get(j) - quantityRemove);
								angularLinearForce.set(j, angularLinearForce.get(j).mult(1-percentRemove));
								angularForce.set(j, angularForce.get(j).mult(1-percentRemove));
							}
							sumPercent -= quantity;
						}else{
							percentAngular.set(i, quantity);
							sumPercent += quantity;
							sumAngular.addLocal(angularVect);
							angularLinearForce.set(i, angularLinearVect);
							angularForce.set(i, angularVect);
						}
					}else{
						//all angular force in the same dir: cool!
						percentAngular.set(i, 1f);
						sumPercent += 1f;
						sumAngular.addLocal(angularVect);
						angularLinearForce.set(i, angularLinearVect);
						angularForce.set(i, angularVect);
					}
				}
			}
		}
		
		//TODO: sumPercent is useful? why i have added it?
		
		System.out.println("FR linear intermediaire : "+sumLinear);
		System.out.println("FR angular intermediaire : "+sumAngular);
		
		if(sumAngular.lengthSquared()>0){
			// on va regarder les points de contacts qui ont une normale dans le meme sens que forceAngul X position
			// Ce sont les points de contacts
			Vector3f bestPoint = null;
			int bestPointIdx = -1;
			float bestDist = 0;
			Plane planRotation = new Plane(sumAngular, 0);
			for (int i=0;i<points.size();i++) {
				Vector3f vect = fPos.subtract(points.get(i));
				float checkOrientation = sumAngular.cross(vect).dot(f.normales.get(pointsIdx.get(i)));
				if (checkOrientation < 0) {
					//check dist
					float val = planRotation.getClosestPoint(vect).lengthSquared();
					if(val > bestDist){
						bestPointIdx = i;
						bestDist = val;
						bestPoint = vect;
					}
				}else{
					System.out.println("point "+i+" @"+points.get(i)+" is not in the right direction for angular pivot");
				}
			}
			rotationPoint = bestPoint;
			// on garde le plus éloigné du centre de l'objet, dans le plan de la rotation.
			if(bestPointIdx>=0){
				// on imagine maintenant que al rotation se fait à ce point.
				// on ajoute une force linéaire FLR au centre de gravité: position du point(norm) X axe de rotation / norme(point)
				// si on veut etre plus précis, on intégre la rotation comme fait dans JointRotation. (TODOAFTER)
				// todo: vérifier la jutification de cela. /\F---C--P-- & \/F-P--C------
				// a mon avis ce n'est pas FC/PC qui font le truc, mais plutot FP/PC
				// en effet (c'est un pb de statique selon wikipedia)
				
//				bestPoint.normalizeLocal().crossLocal(sumAngular)
				
				
				//et voila!
				
				//explication:
				// si FLR dot(newSum) est <=0 => laisser ça comme cela
				// si >0 => il y a une part de la rotation qui est effectué cf ci-dessous
				// si free flight, passer en free flight
				// sinon, laisser l'algo continuer de toute façon... c'est compliqué de voir si il reste collé,
				// 	autant le laisser faire des vas-et-viens entre freeflight et jointPose 
				
				
				//reprenons tout, en version simple:
				// il y a le point d'application de la force F1 en F
				// il y a la force contre laquel on lutte (oupas?) appliqué en C
				// il y a le point pivot de rotation P
				// => il y a une force F2 en C de taille |F1|*FP/PC et de direction -F1
				//
				Vector3f pointRotation = points.get(bestPointIdx);
				Vector3f newSumAngular = new Vector3f(0,0,0);
				
				//pour chaque point, on prend sa force en N (lineaire)
				// on prend le centre de gravité comme opposition
				// on pourra le re-transposer plus tard avec un nouveau centre de rotation (ou le meme)
				for(int i=0; i<f.forces.size(); i++){
					if(percentAngular.get(i)>0){
						float FP = pointRotation.subtract(f.pointApplicationForce.get(i)).length();
						float PC = pointRotation.subtract(fPos).length();
						newSumAngular.addLocal(angularLinearForce.get(i).mult(FP/PC));
						System.out.println("JP angularForce "+angularLinearForce.get(i)+" @"+f.pointApplicationForce.get(i)); 
						System.out.println("JP FP "+FP); 
						System.out.println("JP PC "+PC); 
					}
				}
//				Vector3f linearAtRot = newSumAngular.cross(pointRotation.subtract(fPos));
//				Vector3f linearAtRot = pointRotation.subtract(fPos).cross(newSumAngular);
				Vector3f linearAtRot = newSumAngular;
				
				//add it to linear force ^^ (and let the simu to oscilate from pose/freeflight)
				//TODOAFTER: creer un nouveau lien pour eviter l'oscillation?
				sumLinear.addLocal(linearAtRot);
				//force != acceleration?
				//force lineaire: N (ou kg*m/s²)
				//force (moment) angulaire : N*m (ou kg*m²/s²)
				System.out.println("JP linear UPPPPPP! "+linearAtRot); 
				System.out.println("JP pointRotation "+pointRotation); 
				System.out.println("JP newSumAngular "+newSumAngular); 
				System.out.println("JP pointRotation.subtract(fPos) "+pointRotation.subtract(fPos)); 
			}else{
				//nothing stop it to rotate => rotate!
				sumAngular.divideLocal(f.getIntertiaMoment());
				f.aangulaire.addLocal(sumAngular);
				System.out.println("JP linear rotate! "+sumAngular);
			}
		}
		

//		Vector3f sumForces = sumLinear;
//		Vector3f sumForcesN = sumForces.normalize();
		linearAtGravityCenter.set(sumLinear);
		
		
	}
	
	

}
