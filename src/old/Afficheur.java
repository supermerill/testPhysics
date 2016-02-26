package old;

import java.awt.Color;
import java.awt.Graphics;
import java.awt.event.KeyEvent;
import java.awt.event.KeyListener;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.Set;

import javax.swing.JComponent;
import javax.swing.JFrame;

import jme3Double.Vector3d;
import joint.JointPonctuel;
import joint.JointPose;
import joint.JointRotation;

import collision.CollisionUpdater;

import com.jme3.math.FastMath;
import com.jme3.math.Matrix4f;
import com.jme3.math.Quaternion;
import com.jme3.math.Vector3f;

public class Afficheur extends JComponent {

	ArrayList<Forme> formes = new ArrayList<>();
	Color c = Color.BLACK;
	
	Set<Integer> keyPress = new HashSet<>();

	public static void main(String[] args) {
		JFrame fenetre = new JFrame();
		final Afficheur view = new Afficheur();
		
		Vector3f positionGravite = new Vector3f(0,-100,0);

		Forme f = new Forme("TRAP");
//		f.addPoint(new Vector3f(-15, -15, -6));
//		f.addPoint(new Vector3f(15, -15, -6));
//		f.addPoint(new Vector3f(-15, 15, -6));
//		f.addPoint(new Vector3f(15, 15, -6));
//		f.addPoint(new Vector3f(-3, 3, 6));
//		f.triangles.add(new Triangle(f,0, 1, 2));
//		f.triangles.add(new Triangle(f,1, 2, 3));
//		f.triangles.add(new Triangle(f,0, 1, 4));
//		f.triangles.add(new Triangle(f,1, 2, 4));
//		f.triangles.add(new Triangle(f,2, 3, 4));
//		f.triangles.add(new Triangle(f,3, 0, 4));
//		f.roundBBRayon = 30;
//		//f.pangulaire = new Quaternion().fromAngleAxis(FastMath.PI*32f/180, Vector3f.UNIT_Z);
//		f.acceleration.set(0.0000001f,-0.00000981f,0);
//		//f.vangulaire.set(0,0.001f,0); //1 rotation toute les 5sec
//		f.lastAccel.set(f.acceleration);
//		f.physicUpdate = true;
//		f.landed = false;
//		f.doNotFaceCenter();
//		view.formes.add(f);
		f.addPoint(new Vector3f(-100, 20, -50)); //0
		f.addPoint(new Vector3f(-100, 20, 50)); //1
		f.addPoint(new Vector3f(100, 20, 50)); //2
		f.addPoint(new Vector3f(100, 20, -50)); //3
		f.addPoint(new Vector3f(-100, -20, -50)); //4
		f.addPoint(new Vector3f(-100, -20, 50)); //5
		f.addPoint(new Vector3f(100, -20, 50)); //6
		f.addPoint(new Vector3f(100, -20, -50)); //7
		f.triangles.add(new Triangle(f,1,0,4));
		f.triangles.add(new Triangle(f,4,5,1));
		f.triangles.add(new Triangle(f,2,1,5));
		f.triangles.add(new Triangle(f,5,6,2));
		f.triangles.add(new Triangle(f,3,0,4));
		f.triangles.add(new Triangle(f,4,7,3));
		f.triangles.add(new Triangle(f,0, 1, 2));
		f.triangles.add(new Triangle(f,2, 3, 0));
		f.triangles.add(new Triangle(f,4, 5, 6));
		f.triangles.add(new Triangle(f,6, 7, 4));
		f.triangles.add(new Triangle(f,2,6,7));
		f.triangles.add(new Triangle(f,7,3,2));
		//f.pangulaire = new Quaternion().fromAngleAxis(FastMath.PI*32f/180, Vector3f.UNIT_Z);
		f.forces.add(new Vector3f(0.000000f,-9.81f,0).mult((float)f.mass));
//		f.acceleration.set(0.000000f,-0.00000981f,0);
		//f.vangulaire.set(0,0.001f,0); //1 rotation toute les 5sec
		f.physicUpdate = true;
		f.landed = false;
		f.roundBBRayon = 12;
		f.positionGravite = positionGravite;
		f.position.set(5,0,0);
		for(Vector3f vect : f.points){
			vect.divideLocal(10);
		}
		for(Vector3f vect : f.normales){
			vect.divideLocal(10);
		}
		f.doNotFaceCenter();
		f.computeNormales();
		view.formes.add(f);
		
		//sol

		f = new Forme("SOL");
		f.addPoint(new Vector3f(-200, 10, -100)); //0
		f.addPoint(new Vector3f(-200, 10, 100)); //1
		f.addPoint(new Vector3f(200, 10, 100)); //2
		f.addPoint(new Vector3f(200, 10, -100)); //3
		f.addPoint(new Vector3f(-200, -10, -100)); //4
		f.addPoint(new Vector3f(-200, -10, 100)); //5
		f.addPoint(new Vector3f(200, -10, 100)); //6
		f.addPoint(new Vector3f(200, -10, -100)); //7
		f.triangles.add(new Triangle(f,1,0,4));
		f.triangles.add(new Triangle(f,4,5,1));
		f.triangles.add(new Triangle(f,2,1,5));
		f.triangles.add(new Triangle(f,5,6,2));
		f.triangles.add(new Triangle(f,3,0,4));
		f.triangles.add(new Triangle(f,4,7,3));
		f.triangles.add(new Triangle(f,0, 1, 2));
		f.triangles.add(new Triangle(f,2, 3, 0));
		f.triangles.add(new Triangle(f,4, 5, 6));
		f.triangles.add(new Triangle(f,6, 7, 4));
		f.triangles.add(new Triangle(f,2,6,7));
		f.triangles.add(new Triangle(f,7,3,2));



		f.addPoint(new Vector3f(0, 0, 0)); //8
		f.addPoint(new Vector3f(0, 0, 0)); //9
//		
		f.addPoint(new Vector3f(-33, 10, 0)); //10
		f.addPoint(new Vector3f(20, 10, 20)); //11
		f.addPoint(new Vector3f(20, 10, -20)); //12
		f.addPoint(new Vector3f(0, 55, 0)); //13
//		f.addPoint(new Vector3f(-133, 10, 0)); //10
//		f.addPoint(new Vector3f(-80, 10, 20)); //11
//		f.addPoint(new Vector3f(-80, 10, -20)); //12
//		f.addPoint(new Vector3f(-100, 100, 0)); //13
		f.triangles.add(new Triangle(f,10, 13, 11));
		f.triangles.add(new Triangle(f,11, 13, 12));
		f.triangles.add(new Triangle(f,12, 13, 10));
		f.roundBBRayon = 25;
		f.positionGravite = positionGravite;
		f.position.set(0,-20,0);
		for(Vector3f vect : f.points){
			vect.divideLocal(10);
		}
		for(Vector3f vect : f.normales){
			vect.divideLocal(10);
		}
		f.doNotFaceCenter();
		f.computeNormales();
		view.formes.add(f);

		fenetre.add(view);
		fenetre.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
		fenetre.setSize(800, 500);
		fenetre.setVisible(true);
		
		fenetre.addKeyListener(new KeyListener() {
			
			@Override
			public void keyTyped(KeyEvent e) {
				// TODO Auto-generated method stub
				
			}
			
			@Override
			public void keyReleased(KeyEvent e) {
				view.keyPress.remove(e.getKeyCode());
			}
			
			@Override
			public void keyPressed(KeyEvent e) {
				view.keyPress.add(e.getKeyCode());
			}
		});

		new Thread() {
			public void run() {
				long previousMs = System.currentTimeMillis();
				long nextMs = System.currentTimeMillis();
				CollisionUpdater updater = new CollisionUpdater();
				updater.allFormes = view.formes;
				while (true) {
					
					view.repaint();
					nextMs += 100*5;
					// sleep for 13ms
					if (System.currentTimeMillis() < nextMs) {
						try {
							sleep(nextMs - System.currentTimeMillis());
						} catch (InterruptedException e) {
							e.printStackTrace();
						}
					}
					long currentMs = System.currentTimeMillis();
//					System.out.println("finish sleep: "+currentMs+" =?= "+nextMs);
					int diff = (int)(currentMs-previousMs) /5;
//					System.out.println("diff: "+diff +" = "+currentMs+" - "+previousMs);
					previousMs = currentMs;
					
					if(!view.keyPress.contains(KeyEvent.VK_SPACE)){
						continue;
					}
					
					System.out.println("NEW TURN ---------------------------------------------------------------------------------");
					
					//changements de forces ici!! (user input)
					

					
					//apply gravity
//					ArrayList<Vector3f> force = new ArrayList<>();
//					Vector3f posforce = new Vector3f(0,0,0);
					Forme maForme = view.formes.get(0);
					maForme.forces.clear();
					maForme.pointApplicationForce.clear();
					if(view.keyPress.contains(KeyEvent.VK_Q)){
						maForme.forces.add(new Vector3f(20,20,0));
						maForme.pointApplicationForce.add(maForme.transfoMatrix.mult(new Vector3f(0, 0, 0)));
						maForme.physicUpdate = true;
						System.out.println("add force "+new Vector3f(20,20,0)+" @"+new Vector3f(0, 0, 0));
					}
					if(view.keyPress.contains(KeyEvent.VK_D)){
						maForme.forces.add(new Vector3f(-20,20,0));
						maForme.pointApplicationForce.add(maForme.transfoMatrix.mult(new Vector3f(0, 0, 0)));
						maForme.physicUpdate = true;
						System.out.println("add force "+new Vector3f(-20,20,0)+" @"+new Vector3f(0, 0, 0));
					}
					if(view.keyPress.contains(KeyEvent.VK_Z)){
						maForme.forces.add(new Vector3f(0,20,0));
						maForme.pointApplicationForce.add(maForme.transfoMatrix.mult(new Vector3f(0, 0, 0)));
						maForme.physicUpdate = true;
						System.out.println("add force "+new Vector3f(0,20,0)+" @"+new Vector3f(0, 0, 0));
					}
					if(view.keyPress.contains(KeyEvent.VK_E)){
						maForme.forces.add(new Vector3f(0,20,0));
						maForme.pointApplicationForce.add(maForme.transfoMatrix.mult(new Vector3f(-100, 0, 0)));
						maForme.physicUpdate = true;
						System.out.println("add force "+new Vector3f(0,20,0)+" @"+new Vector3f(-100, 0, 0));
					}
					if(view.keyPress.contains(KeyEvent.VK_R)){
						maForme.forces.add(new Vector3f(0,20,0));
						maForme.pointApplicationForce.add(maForme.transfoMatrix.mult(new Vector3f(100, 0, 0)));
						maForme.physicUpdate = true;
						System.out.println("add force "+new Vector3f(0,20,0)+" @"+new Vector3f(100, 0, 0));
					}
//					for(Forme f : view.formes){
////						while(f.forces.size()>1){
////							f.forces.remove(f.forces.size()-1);
////						}
//						f.forces.clear();
//						f.pointApplicationForce.clear();
//						if(!f.landed){
//							f.forces.add(force);
//							f.pointApplicationForce.add(posforce);
//							System.out.println("add force "+force+" @"+posforce);
//						}
////						f.angularForces.clear();
////						f.angularForces.add(aforce);
//						f.physicUpdate = true;
//					}
					
					
					//calculate force & accel (and some speed)
					for(Forme f : view.formes){
						System.out.println("forme "+f +"@"+f.position);
						if(f.joint instanceof JointPose){
							JointPose joint = (JointPose) f.joint;
							System.out.println("Check affb tri "+f+" : ");
							int numPointToCheckAeff = 0;
							while (numPointToCheckAeff < joint.points.size()) {
									Vector3f point = joint.points.get(numPointToCheckAeff);
									System.out.println("Joint pose already has : " + point + ".distance("
											+ f.transfoMatrix.mult(f.points.get(joint.pointsIdx.get(numPointToCheckAeff))) 
											+ ") ="+point.distance(f.transfoMatrix.mult(f.points.get(joint.pointsIdx.get(numPointToCheckAeff))))+" > 0.0001");
								numPointToCheckAeff ++;
							}
						}
						
						//update forces & accels vector (sometimes even speed ones)
						f.joint.updateForce(currentMs, diff);
						f.joint.updateVitesse(currentMs, diff);
						System.out.println("f.joint.update edned "+f +"@"+f.position +" == "+f.joint.f+"@"
								+f.joint.f.position);
//						f.update(5*diff/2);
					}

					for(Forme f : view.formes){
						System.out.println("forme "+f +"@"+f.position);
						

						if(f.joint instanceof JointPose){
							JointPose joint = (JointPose) f.joint;
							System.out.println("Check affc tri "+f+" : ");
							int numPointToCheckAeff = 0;
							while (numPointToCheckAeff < joint.points.size()) {
									Vector3f point = joint.points.get(numPointToCheckAeff);
									System.out.println("Joint pose already has : " + point + ".distance("
											+ f.transfoMatrix.mult(f.points.get(joint.pointsIdx.get(numPointToCheckAeff))) 
											+ ") ="+point.distance(f.transfoMatrix.mult(f.points.get(joint.pointsIdx.get(numPointToCheckAeff))))+" > 0.0001");
								numPointToCheckAeff ++;
							}
						}
						
					}
					//check collision & update positions
					updater.updateCollision(currentMs, diff);

//					for(Forme f : view.formes){
//						if(!f.landed)
//						for(Forme f2 : view.formes){
//							if(f2.landed){
//								CollisionUpdater.predictRayCollision(f, f2, currentMs);
//								CollisionMobileSol collision = f.checkCollision(f2);
//								if(collision != null){
//									//TODO: placer f tel que le point qui a touch� soit pile 
//									//dans le plan du triangle touch�
//									collision.placeMobile();
//									
//									//TODO: spliter le triangle en cr�ant un nouveau point "identique � l'autre
//									//TODO: "lier" les deux objets via un joint tournant (frottements infini)..oupa
//									
//									//TODO: continuer � le faire r�agir � la pesanteur
//									// => modifier l'algo de maj si li�
//									//		faire le calcul des forces => ajout des r�sultantes => acceleration
//									//		si objet li� �  1 point => si dans la dir de la somme des forces => le garder sinon le retirer et revenir au mode normal
//									//			faire une rotation libre
//									//		si objet li� � deux points & dans la meme dir que force, 
//									//			faire un rotation sur un axe fixe
//									//			en fait, avec une somme des resultantes, pas besoin de deuxi�me cas, ni de troisi�me.
//									
//									f.vitesse.set(Vector3f.ZERO);
//									f.acceleration.set(Vector3f.ZERO);
//									f.lastAccel.set(Vector3f.ZERO);
//									f.landed = true;
//								}
//							}
//						}
//					}
				}
			}
		}.start();
	}

	@Override
	public void paintComponent(Graphics g) {
		super.paintComponent(g);
		paintInside(g);
	}

	private void paintInside(Graphics g) {
		// g.setColor(Color.RED);
		// g.fillOval(0, 0, 400, 400);

		int maxY = getHeight() / 3;
		int maxX = getWidth() / 2;

		int taille = 10;

		g.setColor(c);
//		System.out.println("===DRAW===");
		for (Forme forme : formes) {
//			System.out.println(" =Forme@"+forme.position+" : "+forme.transfoMatrix);
			//create transformation matrix
			g.setColor(c);
			for (Triangle tri : forme.triangles) {

//				System.out.println("m = "+forme.points.get(tri.a));
//				System.out.println("m = "+forme.transfoMatrix.mult(forme.points.get(tri.a), null));
				
				Vector3f m = forme.transfoMatrix.mult(forme.points.get(tri.a), null).multLocal(taille);
				Vector3f n = forme.transfoMatrix.mult(forme.points.get(tri.b), null).multLocal(taille);
				Vector3f e = forme.transfoMatrix.mult(forme.points.get(tri.c), null).multLocal(taille);
				g.drawLine(maxX + (int) (m.x + m.z / 2), maxY
						- (int) (m.y + m.z / 2), maxX + (int) (n.x + n.z / 2),
						maxY - (int) (n.y + n.z / 2));
				g.drawLine(maxX + (int) (e.x + e.z / 2), maxY
						- (int) (e.y + e.z / 2), maxX + (int) (n.x + n.z / 2),
						maxY - (int) (n.y + n.z / 2));
				g.drawLine(maxX + (int) (m.x + m.z / 2), maxY
						- (int) (m.y + m.z / 2), maxX + (int) (e.x + e.z / 2),
						maxY - (int) (e.y + e.z / 2));
			}
			g.setColor(Color.BLUE);
			if(forme.vitesse.lengthSquared() != 0){
				Vector3f m = forme.vitesse.mult(1000).addLocal(forme.position.toVec3f().multLocal(taille));
				Vector3f n = forme.position.toVec3f().multLocal(taille);
				g.drawLine(maxX + (int) (m.x + m.z / 2), maxY
						- (int) (m.y + m.z / 2), maxX + (int) (n.x + n.z / 2),
						maxY - (int) (n.y + n.z / 2));
			}
			g.setColor(Color.RED);
//			for(int i=0;i<forme.normales.size(); i++){
//				Vector3f m = forme.transfoMatrix.mult(forme.points.get(i), null).multLocal(taille);
//				Vector3f n = m.add(forme.normales.get(i).mult(100));
//				g.drawLine(maxX + (int) (m.x + m.z / 2), maxY
//						- (int) (m.y + m.z / 2), maxX + (int) (n.x + n.z / 2),
//						maxY - (int) (n.y + n.z / 2));
//			}
			

			if(forme.vangulaire.lengthSquared() != 0){
				Vector3f m = forme.vangulaire.mult(1000).addLocal(forme.position.toVec3f().multLocal(taille));
				Vector3f n = forme.position.toVec3f().multLocal(taille);
				//draw the rotaion vector?
//				g.drawLine(maxX + (int) (m.x + m.z / 2), maxY
//						- (int) (m.y + m.z / 2), maxX + (int) (n.x + n.z / 2),
//						maxY - (int) (n.y + n.z / 2));
				for (Vector3f point : forme.points) {
					m = forme.transfoMatrix.mult(point, null).multLocal(taille);
					Vector3f om = m.subtract(forme.position.toVec3f().multLocal(taille));
					Vector3f rotVect = forme.vangulaire.mult(100).crossLocal(om);
					n = rotVect.addLocal(m);
					//compound vith velocity?
					n.addLocal(forme.vitesse.mult(100));
					//draw each point of rotation
					g.drawLine(maxX + (int) (m.x + m.z / 2), maxY
							- (int) (m.y + m.z / 2), maxX + (int) (n.x + n.z / 2),
							maxY - (int) (n.y + n.z / 2));

				}
				
			}
			
			g.setColor(Color.BLUE);
			Vector3f pc = new Vector3f();
			if(forme.joint instanceof JointRotation){
				pc.set(((JointRotation)forme.joint).pointWRotation).multLocal(taille);
				g.fillOval((int) (maxX + pc.x + pc.z / 2) - 2, (int) (maxY - pc.y - pc.z / 2) - 2, 5, 5);
			}
			g.setColor(Color.MAGENTA);
			if(forme.joint instanceof JointPose){
				for(Vector3f v : ((JointPose)forme.joint).points){
					pc.set(v).multLocal(taille);
					g.fillOval((int) (maxX + pc.x + pc.z / 2) - 2, (int) (maxY - pc.y - pc.z / 2) - 2, 5, 5);
				}
			}
			
		}

	}

}
