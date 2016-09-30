#include <osgViewer/Viewer>
#include <osgDB/ReadFile>
#include <osgText/Font>
#include <osgText/Text>
#include <osg/MatrixTransform>
#include <osgGA/TrackballManipulator>
#include <bullet/btBulletDynamicsCommon.h>
#include <osgShadow/ShadowedScene>
#include <osgShadow/ShadowMap>
 #include <osgUtil/Optimizer>
 #include <osgParticle/PrecipitationEffect>
 

int ResetFlag=0;
int count=0;
btRigidBody* pinoRigidBody[10];
btRigidBody* ballRigidBody;
btRigidBody* pranchaRigidBody = NULL;
btRigidBody* barreiraRigidBody = NULL;
osgText::Text* text;
	int maxcount=0;
osg::Vec3 bt2osg_Vec3(btVector3 bv) {
	return osg::Vec3( bv.x(), bv.y(), bv.z() );
}
osg::Vec4 bt2osg_Vec4(btVector4 bv) {
	return osg::Vec4( bv.x(), bv.y(), bv.z(), bv.w() );
}
btVector3 osg2bt_Vec3(osg::Vec3 bv) {
	return btVector3( bv.x(), bv.y(), bv.z() );
}
osg::Quat bt2osg_Quat(btQuaternion bv) {
	return osg::Quat( bv.x(), bv.y(), bv.z(), bv.w() );
}


// class to handle events
class EventHandler : public osgGA::GUIEventHandler
{
	public:
		bool handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
		{
			osgViewer::Viewer* viewer = dynamic_cast<osgViewer::Viewer*>(&aa);
			if (!viewer) return false;
			switch(ea.getEventType())
			{
				case(osgGA::GUIEventAdapter::KEYUP):
					switch ( ea.getKey() ) 
					{
						case 'S':
							std::cout << "tecla S" << std::endl;
							return false;
						case 'r':
							ResetFlag = 3;
							maxcount=0;
							std::cout << "Reset" << std::endl;
							break;
						// para empurrar pinos (sem uso da bola)
						case 'k':
							for(int p=0 ; p<10 ; p++ ) 
							{
								//pinoRigidBody[p]->applyForce(btVector3(100,-100,10),btVector3(0,0,0) );
								pinoRigidBody[p]->setLinearVelocity( btVector3(0,0,-100) );
							}

							break;
					}
				// mover a prancha de acordo com o rato
				case(osgGA::GUIEventAdapter::MOVE):
						if ( pranchaRigidBody)
						{
						btTransform trans;
						pranchaRigidBody->getMotionState()->getWorldTransform(trans);
						trans.setRotation(btQuaternion((double)(ea.getX()-500.)/1000.,-(double)(ea.getY()-300.)/1000., 0.) );
						pranchaRigidBody->getMotionState()->setWorldTransform(trans);
						}

					return false;
				default:
					return false;
			}
		}
};
osg::MatrixTransform *AddBox(osg::Group* upNode, btCompoundShape* upShape,float sx, float sy, float sz, float x0, float y0, float z0)
{
	osg::Matrix myMatrix;
	osg::Node* loadedModel = osgDB::readNodeFile("cube.obj");
	// Graphical Node
	osg::MatrixTransform* myTransform = new osg::MatrixTransform;
	myMatrix = osg::Matrix::scale(sx/2., sy/2., sz/2.);
	myMatrix.setTrans( x0, y0, z0);
	myTransform->setMatrix( myMatrix );
	myTransform->addChild(loadedModel);

	// criar sombras
	osg::ref_ptr<osg::Material> mat = new osg::Material;
	 mat->setColorMode(osg::Material::DIFFUSE);
	 mat->setAmbient (osg::Material::FRONT_AND_BACK, osg::Vec4(0.2, 20.2, 10.2, 1.0));
	 mat->setDiffuse (osg::Material::FRONT_AND_BACK, osg::Vec4(0.9, 0.3, 10.4, 1.0));
	 mat->setSpecular(osg::Material::FRONT_AND_BACK, osg::Vec4(1.0, 1.0, 1.0, 1.0));
	 mat->setShininess(osg::Material::FRONT_AND_BACK, 4);
	 myTransform->getOrCreateStateSet()->
	 setAttributeAndModes(mat.get(), osg::StateAttribute::ON);


	upNode->addChild(myTransform);
	//Collision shape
	btCollisionShape* boxShape = new btBoxShape(btVector3(sx/2.,sy/2.,sz/2.));
	upShape->addChildShape(btTransform(btQuaternion(0,0,0,1),btVector3(x0,y0,z0)), boxShape );
	return myTransform;
}
// funçao de reset dos pinos
void Reset() {
	if (ResetFlag==0) return;
		std::cout << "Reset" << std::endl;
	int p;
	int l=1, ln=1, lp=0;
	float px=1., py;
	if ( ResetFlag==1 ) 
	{
		ballRigidBody->setCollisionFlags( pinoRigidBody[p]->getCollisionFlags() &! btCollisionObject::CF_KINEMATIC_OBJECT );
		for( p=0 ; p<10 ; p++ )
			pinoRigidBody[p]->setCollisionFlags( pinoRigidBody[p]->getCollisionFlags() &! btCollisionObject::CF_KINEMATIC_OBJECT );
	}
	if ( ResetFlag==3 ) {

		printf("\7\7\7");
		count=0;
		btTransform trans;
		ballRigidBody->setCollisionFlags( ballRigidBody->getCollisionFlags() | 
			btCollisionObject::CF_KINEMATIC_OBJECT );
		ballRigidBody->clearForces();
		ballRigidBody->setLinearVelocity( btVector3(0,0,0) );
		ballRigidBody->setAngularVelocity( btVector3(0,0,0) );
		ballRigidBody->getMotionState()->getWorldTransform(trans);
		trans.setRotation(btQuaternion(0,0,0,1));
		trans.setOrigin(btVector3(-3.,0,5.));
		// Posicao inicial da bola
		ballRigidBody->getMotionState()->setWorldTransform(trans);
		for( p=0 ; p<10 ; p++ ) 
		{
			pinoRigidBody[p]->setCollisionFlags( pinoRigidBody[p]->getCollisionFlags() | btCollisionObject::CF_KINEMATIC_OBJECT );
			pinoRigidBody[p]->clearForces();
			pinoRigidBody[p]->setLinearVelocity( btVector3(0,0,0) );
			pinoRigidBody[p]->setAngularVelocity( btVector3(0,0,0) );
			pinoRigidBody[p]->getMotionState()->getWorldTransform(trans);
			lp++;
			if ( lp>ln ) 
			{ 
				l++; lp=1; ln++; px=(float)l; 
			}
			py = -0.5 + (float)lp - ((float)l)/2.;
			trans.setRotation(btQuaternion(0,0,0,1));
			trans.setOrigin(btVector3(px+2.5,py,1));
			pinoRigidBody[p]->getMotionState()->setWorldTransform(trans);
		}
	}
	ResetFlag--;
}

int main()
{

	// Create dynamic world
	btBroadphaseInterface* broadphase = new btDbvtBroadphase();
	btDefaultCollisionConfiguration* collisionConfiguration = new btDefaultCollisionConfiguration();
	btCollisionDispatcher* dispatcher = new btCollisionDispatcher(collisionConfiguration);
	btSequentialImpulseConstraintSolver* solver = new btSequentialImpulseConstraintSolver;
	btDiscreteDynamicsWorld* dynamicsWorld = new btDiscreteDynamicsWorld(dispatcher,broadphase,solver,collisionConfiguration);

	// Set Gravity
	dynamicsWorld->setGravity(btVector3(0.,0.,-9.8));


	double z_bola = 5.;
	double v_bola = 0.;
	osg::Matrix myMatrix;

	// Creating the root node
	osg::Group* SceneRoot = new osg::Group;
	
	// Add Light Source
	osg::LightSource *ls = new osg::LightSource;
	ls->getLight()->setPosition(osg::Vec4(0.5,-0.7,1.,0.));
	ls->getLight()->setAmbient(osg::Vec4(0.2,0.2,0.2,1));
	SceneRoot->addChild( ls );

	 osgShadow::ShadowedScene * shadowScene = new osgShadow::ShadowedScene;
	 osgShadow::ShadowMap * sm = new osgShadow::ShadowMap;
	 shadowScene->setShadowTechnique(sm);
	 shadowScene->addChild(ls);
	 SceneRoot->addChild(shadowScene);
	// Bola
        osg::Node* loadedModel = osgDB::readNodeFile("bola.obj");

		btCollisionShape* ballShape = new btSphereShape(0.5);
		btDefaultMotionState* ballMotionState = new btDefaultMotionState(btTransform(btQuaternion(0,0,0,1),btVector3(-6.,0.,z_bola)));
		btScalar mass = 1;
		btVector3 ballInertia(0,0,0);
		ballShape->calculateLocalInertia(mass,ballInertia);
		btRigidBody::btRigidBodyConstructionInfo ballRigidBodyCI(mass,ballMotionState,ballShape,ballInertia);
		ballRigidBody = new btRigidBody(ballRigidBodyCI);
		
		dynamicsWorld->addRigidBody(ballRigidBody);
		ballRigidBodyCI.m_restitution = 0.95f;

		
 	
	// bolaPos
		osg::MatrixTransform* bolaPos = new osg::MatrixTransform;
		z_bola = 10.;
		bolaPos->addChild( loadedModel );
		myMatrix.setTrans( 0., 0.,z_bola);
		bolaPos->setMatrix( myMatrix );
		bolaPos->addChild( loadedModel );
		shadowScene->addChild(bolaPos);	
	
	/* pino
		float raio = 0.07;
		float altura = 0.4;
		mass = 0.3;
		osg::Matrix pinoMatrix;
		pinoMatrix.setTrans( 0., 0, altura/2.);
		osg::MatrixTransform* pinoPos = new osg::MatrixTransform;
		pinoPos->setMatrix( pinoMatrix );
		loadedModel = osgDB::readNodeFile("pino.obj");
		pinoPos->addChild(loadedModel);
		SceneRoot->addChild(pinoPos);

			btCollisionShape* pinoShape = new
			btCylinderShapeZ(btVector3(raio,raio,altura/2.));
			btDefaultMotionState* pinoMotionState = new
			btDefaultMotionState(btTransform(btQuaternion(0,0,0,1),
			btVector3(0,0,altura/2.)));
			btVector3 pinoInertia(0,0,0);
			pinoShape->calculateLocalInertia(mass,pinoInertia);
			btRigidBody::btRigidBodyConstructionInfo
			pinoRigidBodyCI(mass,pinoMotionState,pinoShape,pinoInertia);
			pinoRigidBodyCI.m_restitution = 0.55f;
			pinoRigidBodyCI.m_friction = 0.8f;
			btRigidBody* pinoRigidBody = new btRigidBody(pinoRigidBodyCI);
			dynamicsWorld->addRigidBody(pinoRigidBody);*/

	// 10 pinos
		float raio = 0.35;
		float altura = 2.;
		mass = 0.09;
		loadedModel = osgDB::readNodeFile("pino.obj");
		btVector3 pinoInertia(0,0,0);
		//btCollisionShape* pinoShape = new btCylinderShapeZ(btVector3(raio,raio,altura/2.));
			btCompoundShape* pinoShape = new btCompoundShape();
			pinoShape->addChildShape(btTransform(btQuaternion(0,0,0,1),btVector3(0,0,0)), new btCylinderShapeZ(btVector3(raio,raio,altura/2)) );
			pinoShape->addChildShape(btTransform(btQuaternion(0,0,0,1),btVector3(0,0,0.4)), new btSphereShape(0.36) );
			pinoShape->addChildShape(btTransform(btQuaternion(0,0,0,1),btVector3(0,0,altura-0.25)), new btSphereShape(0.25) );
		//btCollisionShape* pinoShape = new 
		pinoShape->calculateLocalInertia(mass,pinoInertia);
		osg::MatrixTransform* pinoPos[10];
		
		int p;
		int l=1, ln=1, lp=0;
		float px=1., py=1;
		for( p=0 ; p<10 ; p++ ) 
		{
			pinoPos[p] = new osg::MatrixTransform;
			pinoPos[p]->addChild(loadedModel);
			shadowScene->addChild(pinoPos[p]);
			lp++;
			if ( lp>ln ) { l++; lp=1; ln++; px=(float)l; }
			py = -0.5 + (float)lp - ((float)l)/2.;
			btDefaultMotionState* pinoMotionState = new
			btDefaultMotionState(btTransform(btQuaternion(0,0,0,1),btVector3(px+2.5,py,altura/2.)));
			btRigidBody::btRigidBodyConstructionInfo
			pinoRigidBodyCI(mass,pinoMotionState,pinoShape,pinoInertia);
			pinoRigidBodyCI.m_restitution = 0.95f;
			pinoRigidBodyCI.m_friction = 0.8f;
			pinoRigidBody[p] = new btRigidBody(pinoRigidBodyCI);

			dynamicsWorld->addRigidBody(pinoRigidBody[p]);
		}

	// Ground
		osg::MatrixTransform *myTransform = new osg::MatrixTransform;
		myMatrix = osg::Matrix::rotate(0, osg::Vec3(0.,1.,0.) );
		osg::Vec3 myNorm = myMatrix.preMult( osg::Vec3(0.,0.,1.) ) ;
		double z0 = 0;
		myMatrix.setTrans( 0., 0, z0);
		myTransform->setMatrix( myMatrix );
		loadedModel = osgDB::readNodeFile("plano.obj");
		myTransform->addChild(loadedModel);
		shadowScene->addChild(myTransform);

			btCollisionShape* groundShape = new btStaticPlaneShape(osg2bt_Vec3(myNorm), z0);
			btDefaultMotionState* groundMotionState = new 
				btDefaultMotionState(btTransform(btQuaternion(0,0,0,1),btVector3(0,0,0)));
			btRigidBody::btRigidBodyConstructionInfo
				groundRigidBodyCI(0,groundMotionState,groundShape,btVector3(0,0,0));
			btRigidBody* groundRigidBody = new btRigidBody(groundRigidBodyCI);
			dynamicsWorld->addRigidBody(groundRigidBody);
			groundRigidBodyCI.m_restitution = 0.95f;
			groundRigidBodyCI.m_friction = 0.2f;

		// Prancha
		osg::MatrixTransform* pranchaTransform = new osg::MatrixTransform;
		btCompoundShape* pranchaShape = new btCompoundShape();
		AddBox(pranchaTransform, pranchaShape, 10., 10., 0.1,-5., 0., 1.);
		AddBox(pranchaTransform, pranchaShape, 10., 1., 1.,-5., 5.,1.5);
		AddBox(pranchaTransform, pranchaShape, 10., 1., 1.,-5., -5.,1.5);
		AddBox(pranchaTransform, pranchaShape, 1., 9., 1.,-9.5, 0.,1.5);
		AddBox(pranchaTransform, pranchaShape, 3., 1., 1.,-8.5, -2.,1.5);
		AddBox(pranchaTransform, pranchaShape, 3., 1., 1.,-8.5, 2.,1.5);
		AddBox(pranchaTransform, pranchaShape, 1., 2., 1.,	-0.5, -3.5,1.5);
		AddBox(pranchaTransform, pranchaShape, 1., 2., 1.,	-0.5, 3.5,1.5);
		AddBox(pranchaTransform, pranchaShape, 5., 1., 1.,	-2.5, -2.,1.5);
		AddBox(pranchaTransform, pranchaShape, 4., 1., 1.,	-2., 2.,1.5);
		AddBox(pranchaTransform, pranchaShape, 1., 4., 1.,	-5.5, -0.5,1.5);

		shadowScene->addChild(pranchaTransform);
		btDefaultMotionState* pranchaMotionState = new btDefaultMotionState(btTransform(btQuaternion(0,0,0,1),btVector3(0,0,0)));
		btRigidBody::btRigidBodyConstructionInfo
			pranchaRigidBodyCI(0.,pranchaMotionState,pranchaShape,btVector3(0,0,0));
		pranchaRigidBodyCI.m_restitution =0.95f;
		pranchaRigidBodyCI.m_friction = 0.2f;
		pranchaRigidBody = new btRigidBody(pranchaRigidBodyCI);
		pranchaRigidBody->setCollisionFlags( pranchaRigidBody->getCollisionFlags() |
			btCollisionObject::CF_KINEMATIC_OBJECT );
		pranchaRigidBody->setActivationState(DISABLE_DEACTIVATION);
		dynamicsWorld->addRigidBody(pranchaRigidBody);

		//barreiras
		osg::MatrixTransform* barreiraTransform = new osg::MatrixTransform;
		btCompoundShape* barreiraShape = new btCompoundShape();
		AddBox(barreiraTransform, barreiraShape, 10., 0.1, 1., 5., 5., 0.5);
		AddBox(barreiraTransform, barreiraShape, 10., 0.1, 1., 5., -5., 0.5);
		AddBox(barreiraTransform, barreiraShape, 0.1, 10., 2.5, 10., 0., 0.5);

		shadowScene->addChild(barreiraTransform);
		btDefaultMotionState* barreiraMotionState = new btDefaultMotionState(btTransform(btQuaternion(0,0,0,1),btVector3(0,0,0)));
		btRigidBody::btRigidBodyConstructionInfo
			barreiraRigidBodyCI(0.,barreiraMotionState,barreiraShape,btVector3(0,0,0));
		barreiraRigidBodyCI.m_restitution =0.95f;
		barreiraRigidBodyCI.m_friction = 0.2f;
		barreiraRigidBody = new btRigidBody(barreiraRigidBodyCI);
		barreiraRigidBody->setCollisionFlags( barreiraRigidBody->getCollisionFlags() |
			btCollisionObject::CF_KINEMATIC_OBJECT );
		barreiraRigidBody->setActivationState(DISABLE_DEACTIVATION);
		dynamicsWorld->addRigidBody(barreiraRigidBody);

	/*// Ground - Rampa
		osg::MatrixTransform *myTransform2 = new osg::MatrixTransform;
		myMatrix = osg::Matrix::rotate(0.25, osg::Vec3(0.,1.,0.) );
		osg::Vec3 myNorm2 = myMatrix.preMult( osg::Vec3(0.,0.,1.) ) ;
		double z1 = 0;
		myMatrix.setTrans( -1., 0., z1);
		myTransform2->setMatrix( myMatrix );
		loadedModel = osgDB::readNodeFile("plano.obj");
		myTransform2->addChild(loadedModel);
		SceneRoot->addChild(myTransform2);

			btCollisionShape* groundShape2 = new btStaticPlaneShape(osg2bt_Vec3(myNorm2), z1);
			btDefaultMotionState* groundMotionState2 = new 
				btDefaultMotionState(btTransform(btQuaternion(0,0,0,1),btVector3(0,0,0)));
			btRigidBody::btRigidBodyConstructionInfo
				groundRigidBodyCI2(0,groundMotionState2,groundShape2,btVector3(0,0,0));
			btRigidBody* groundRigidBody2 = new btRigidBody(groundRigidBodyCI2);
			dynamicsWorld->addRigidBody(groundRigidBody2);
			groundRigidBodyCI2.m_restitution = 0.95f;
			groundRigidBodyCI2.m_friction = 0.2f;*/


	 osgUtil::Optimizer optimizer;
	 optimizer.optimize( SceneRoot );
	 	text 	= new osgText::Text();
	 	char aux[50];
	 	sprintf(aux,"Pinos caidos %d",count);
		text->setText(aux); 
        text->setCharacterSize(1.0f);
		text->setPosition(osg::Vec3(-6,-5,10.0f));
		text->setAxisAlignment(osgText::Text::XZ_PLANE); 
		osg::Geode* geode  = new osg::Geode;
		geode->addDrawable(text);
       shadowScene->addChild(geode);
	
	
	
	// Creating the viewer
	osgViewer::Viewer viewer ;
	viewer.setSceneData( SceneRoot );

	// add the Event handler
	viewer.addEventHandler(new EventHandler());

	// Setup camera
	osg::Matrix matrix;
 	matrix.makeLookAt( osg::Vec3(0.,-30.,15.), osg::Vec3(0.,0.,5.), osg::Vec3(0.,0.,1.) );
 	viewer.getCamera()->setViewMatrix(matrix);
	

        //viewer.setCameraManipulator(  new osgGA::TrackballManipulator() );

	// record the timer tick at the start of rendering.
	osg::Timer myTimer;
	double time_now = myTimer.time_s();
    	double last_time = time_now;
	double frame_time = 0.;
	double F;
	int m = 1;
	double g = -9.8;
	double a;
	double dt;
	printf("\7\7\7");
	//ciclo de processamento
    while( !viewer.done() )
  	{

		Reset();
	 	char aux[50];

							
		if(count>maxcount)
		{

	 		maxcount=count;
		}
	 	if(maxcount==10)
	 	sprintf(aux,"Strike!!");
	 	else
	 	sprintf(aux,"Pinos caidos %d",maxcount);
		text->setText(aux); 
		count=0;

	
	     	viewer.frame();
		time_now = myTimer.time_s();
		frame_time = time_now - last_time;
		last_time = time_now;
		// z = z0+ v0*t + 0.5*a*t2
		double raio_bola = 0.5;
		//transformações fisicas da bola
		if( z_bola-raio_bola<z0) 
		{
			z_bola = z0+raio_bola;
			v_bola =-v_bola;
		}
		z_bola = 10.-0.5*9.8*time_now*time_now;
		dt = frame_time;
		F = m*g;
		a = F / m;
		v_bola += a*dt;
		z_bola += v_bola*dt;
		//myMatrix.makeTranslate(0., 0., z_bola);
		bolaPos->setMatrix(myMatrix);
		dynamicsWorld->stepSimulation(frame_time, 10);
		btTransform trans;
		ballRigidBody->getMotionState()->getWorldTransform(trans);
		myMatrix.makeRotate(bt2osg_Quat(trans.getRotation()));
		myMatrix.setTrans(bt2osg_Vec3(trans.getOrigin()));
		bolaPos->setMatrix(myMatrix);
		if(trans.getOrigin().getX()>=9.4 && trans.getOrigin().getZ()>20)
		{
			ResetFlag=3;
			maxcount=0;
		}

		// verificar pinos caidos.
		for( p=0 ; p<10 ; p++ ) {
		pinoRigidBody[p]->getMotionState()->getWorldTransform(trans);
		matrix.makeRotate(bt2osg_Quat(trans.getRotation()));
		matrix.setTrans(bt2osg_Vec3(trans.getOrigin()));
		pinoPos[p]->setMatrix(matrix);
		if(trans.getOrigin().getZ()<0.5)
			count++;
		   
		}
		
		/*pinoRigidBody->getMotionState()->getWorldTransform(trans);
		matrix.makeRotate(bt2osg_Quat(trans.getRotation()));
		matrix.setTrans(bt2osg_Vec3(trans.getOrigin()));
		pinoPos->setMatrix(matrix);*/
		pranchaRigidBody->getMotionState()->getWorldTransform(trans);
		myMatrix.makeRotate(bt2osg_Quat(trans.getRotation()));
		myMatrix.setTrans(bt2osg_Vec3(trans.getOrigin()));
		pranchaTransform->setMatrix(myMatrix);

  	}
}
