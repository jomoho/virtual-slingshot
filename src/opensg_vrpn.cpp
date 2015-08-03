#include <cstdlib>
#include <cstddef>
#include <cmath>
#include <iostream>
#include <ios>
#include <ctime>

#include <OpenSG/OSGGLUT.h>
#include <OpenSG/OSGConfig.h>
#include <OpenSG/OSGSimpleGeometry.h>
#include <OpenSG/OSGGLUTWindow.h>
#include <OpenSG/OSGMultiDisplayWindow.h>
#include <OpenSG/OSGSceneFileHandler.h>
#include <OpenSG/OSGSimpleSceneManager.h>
#include <OpenSG/OSGSimpleGeometry.h>
#include <OpenSG/OSGComponentTransform.h>
#include <OpenSG/OSGMaterialGroup.h>
#include <OpenSG/OSGImage.h>
#include <OpenSG/OSGSimpleTexturedMaterial.h>
#include <OpenSG/OSGSceneFileHandler.h>
#include <OpenSG/OSGTextureBackground.h>
#include <OpenSG/OSGGradientBackground.h>
#include <OpenSG/OSGPointLight.h>
#include <OpenSG/OSGSpotLight.h>

//#include <inVRs/tools/libraries/Skybox/Skybox.h>

#include <OSGCSM/OSGCAVESceneManager.h>
#include <OSGCSM/OSGCAVEConfig.h>
#include <OSGCSM/appctrl.h>

#include <vrpn_Tracker.h>
#include <vrpn_Button.h>
#include <vrpn_Analog.h>



OSG_USING_NAMESPACE


#define MYTEST 

OSGCSM::CAVEConfig cfg;
OSGCSM::CAVESceneManager *mgr = nullptr;
vrpn_Tracker_Remote* tracker =  nullptr;
vrpn_Button_Remote* button = nullptr;
vrpn_Analog_Remote* analog = nullptr;

NodeRecPtr root, 
		landTrans,
		standTrans,
		targetTrans,
		shotTrans,
		projectileTrans;


NodeRecPtr beachTrans;

auto head_orientation = Quaternion(Vec3f(0.f, 1.f, 0.f), 3.141f);
auto head_position = Vec3f(0.f, 170.f, 200.f);	// a 1.7m Person 2m in front of the scene

auto schleuder_orientation = Quaternion();
auto schleuder_position = Vec3f();

auto hand_orientation = Quaternion();
auto hand_position = Vec3f();

//Skybox skybox;

class Projectile {
public: 
	NodeRecPtr node;
	NodeRecPtr	tr;
	Vec3f dir;
	void init(Vec3f pos, Vec3f dir_){

		node = makeSphere(2, 3);
		
		root->addChild(node);
		
		root->subChild(node);
		
		ComponentTransformRecPtr ct = ComponentTransform::create();
		ct->setTranslation(pos);
		ct->setRotation(Quaternion(Vec3f(1,0,0),osgDegree2Rad(0)));

		tr = Node::create();
		tr->setCore(ct);
		tr->addChild(node);
		root->addChild(tr);

		dir = dir_;
	}

	void update(int ticks){
		Vec3f ad = dir * ((float) ticks) *0.01f;
	}
};

void cleanup()
{
	delete mgr;
	delete tracker;
	delete button;
	delete analog;

	beachTrans = NULL;
	mgr = NULL;
	root = NULL; 
	landTrans = NULL;
	standTrans = NULL;
	targetTrans = NULL;
	shotTrans = NULL;
	projectileTrans = NULL;
}

void print_tracker();

void initSkybox() {
std::string skyPath ="models/ThickCloudsWater/ThickCloudsWater";
/*
skybox.init(5,5,5, 1000, (skyPath+"Down2048.png").c_str(),
		(skyPath+"Up2048.png").c_str(),
		(skyPath+"Front2048.png").c_str(),
		(skyPath+"Back2048.png").c_str(),
		(skyPath+"Right2048.png").c_str(),
		(skyPath+"Left2048.png").c_str());
		*/
}

NodeRecPtr loadModel(std::string filename, 
	Vec3f trans = Vec3f(0,0,0), 
	float scale = 1.0f, 
	Quaternion rot = Quaternion(Vec3f(1,0,0),osgDegree2Rad(0))) {

		/*

	//model taken from http://storage3d.com/
	NodeRecPtr palmTree = SceneFileHandler::the()->read("models/landscape.obj");

	ComponentTransformRecPtr palmCT = ComponentTransform::create();
	palmCT->setTranslation(Vec3f(0,0,0));
	palmCT->setRotation(Quaternion(Vec3f(0,1,0),osgDegree2Rad(0)));
	//palmCT->setScale(Vec3f(10.f,10.f,10.f));

	NodeRecPtr palmTrans = makeNodeFor(palmCT);

	palmTrans->addChild(palmTree);
	
	*/
	
 	NodeRecPtr model = SceneFileHandler::the()->read(filename.c_str());

	ComponentTransformRecPtr modelCT = ComponentTransform::create();
	modelCT->setTranslation(trans);
	modelCT->setRotation(rot);
	modelCT->setScale( Vec3f(scale, scale, scale));

	NodeRecPtr modelTrans = makeNodeFor(modelCT);
	modelTrans->addChild(model);

	return modelTrans;
 }

NodeTransitPtr createScenegraph() {
	NodeRecPtr root = Node::create();
	root->setCore(Group::create());

	NodeRecPtr boxChild = makeBox(5,4,4,1,1,1);
	NodeRecPtr beach = makePlane(30000, 30000, 1, 1);
	//NodeRecPtr beach = SceneFileHandler::the()->read("models/landscape.obj");

	GeometryRecPtr sunGeo = makeSphereGeo(2, 3);
	NodeRecPtr sunChild = Node::create();
	sunChild->setCore(sunGeo);

	root->addChild(sunChild);
	root->addChild(boxChild);
	root->addChild(beach);

	//decouple the nodes to be shifted in hierarchy from the scene
	root->subChild(sunChild);
	root->subChild(beach);

	TransformRecPtr sunTransCore = Transform::create();
	Matrix sunMatrix;

	// Setting up the matrix
	sunMatrix.setIdentity();
	sunMatrix.setTranslate(0,20000,0);
	sunTransCore->setMatrix(sunMatrix); // Adding the Matrix to the core

	// Setting up the node
	NodeRecPtr sunTrans = makeNodeFor(sunTransCore);
	sunTrans->addChild(sunChild);

	ComponentTransformRecPtr ct = ComponentTransform::create();
	ct->setTranslation(Vec3f(0,-2,0));
	ct->setRotation(Quaternion(Vec3f(1,0,0),osgDegree2Rad(90)));

	beachTrans = Node::create();
	beachTrans->setCore(ct);
	beachTrans->addChild(beach);

	// put the nodes in the scene again
	//root->addChild(beachTrans);
	root->addChild(sunTrans);
	root->subChild(sunTrans);

	SimpleMaterialRecPtr sunMat = SimpleMaterial::create();
	sunMat->setDiffuse(Color3f(1,0.8f,0));
	sunMat->setAmbient(Color3f(0.8f, 0.2f, 0.2f));

	MaterialGroupRecPtr sunMgCore = MaterialGroup::create();
	sunMgCore->setMaterial(sunMat);

	NodeRecPtr sunMg = Node::create();

	sunMg->setCore(sunMgCore);
	sunMg->addChild(sunTrans);

	root->addChild(sunMg);

	SimpleMaterialRecPtr boxMat = SimpleMaterial::create();

	boxMat->setDiffuse(Color3f(1,0.2f,0.1f));
	boxMat->setAmbient(Color3f(0.8f, 0.2f, 0.2f));
	boxMat->setTransparency(0.25);
	//boxMat->setLit(false);

	GeometryRecPtr boxGeo = dynamic_cast<Geometry*>(boxChild->getCore());
	boxGeo->setMaterial(boxMat);

	ImageRecPtr image = Image::create();
	// sand taken from http://www.filterforge.com/filters/720.jpg
	image->read("models/ground_1024_raw.jpg");

	//now we create the texture that will hold the image
	SimpleTexturedMaterialRecPtr tex = SimpleTexturedMaterial::create();
	tex->setImage(image);

	//now assign the fresh texture to the geometry
	GeometryRecPtr beachGeo = dynamic_cast<Geometry*>(beach->getCore());
	beachGeo->setMaterial(tex);

	landTrans = loadModel("models/landscape.obj");
	root->addChild(landTrans);
	/*
	NodeRecPtr palmTree2 = OSG::deepCloneTree(palmTrans);
	ComponentTransformRecPtr palmCT2 = dynamic_cast<ComponentTransform*>(palmTree2->getCore());

	palmCT2->setTranslation(Vec3f(10,-1,5));
	palmCT->setRotation(Quaternion(Vec3f(1,0,0),osgDegree2Rad(0)));
	palmCT->setScale(Vec3f(10.f,10.f,10.f));

	root->addChild(palmTree2);
	*/
	PointLightRecPtr sunLight = PointLight::create();
	//sunLight->setAttenuation(1,0,2);

	//color information
	sunLight->setDiffuse(Color4f(1,1,1,1));
	sunLight->setAmbient(Color4f(0.2f,0.2f,0.2f,1));
	sunLight->setSpecular(Color4f(1,1,1,1));

	sunLight->setBeacon(sunChild); //attach to the sun node use this node as position beacon

	root->setCore(sunLight);

	DirectionalLightRecPtr dirLight = DirectionalLight::create();
	dirLight->setDirection(1,1,-1);

	//color information
	dirLight->setDiffuse(Color4f(1,1,1,1));
	dirLight->setAmbient(Color4f(0.2f,0.2f,0.2f,1));
	dirLight->setSpecular(Color4f(1,1,1,1));


	//wrap the root, cause only nodes below the lights will be lit
	NodeRecPtr ueberroot = makeNodeFor(dirLight);
	ueberroot->addChild(root);

	root = ueberroot;
	return NodeTransitPtr(root);
}

NodeTransitPtr buildScene() {
	//root = Node::create();
	/*
	root->setCore(Group::create());

	GeometryRecPtr sunGeo = makeSphereGeo(2, 3);
	NodeRecPtr sunChild = Node::create();
	sunChild->setCore(sunGeo);
	root->addChild(sunChild);


	//decouple the nodes to be shifted in hierarchy from the scene
	root->subChild(sunChild);

	TransformRecPtr sunTransCore = Transform::create();
	Matrix sunMatrix;

	// Setting up the matrix
	sunMatrix.setIdentity();
	sunMatrix.setTranslate(0,20,0);
	sunTransCore->setMatrix(sunMatrix); // Adding the Matrix to the core

	// Setting up the node
	NodeRecPtr sunTrans = makeNodeFor(sunTransCore);
	sunTrans->addChild(sunChild);


	// put the nodes in the scene again
	root->addChild(sunTrans);
	root->subChild(sunTrans);
	
	//landTrans = loadModel("models/landscape.obj");
	//standTrans = loadModel("models/stand.obj");
	
	//targetTrans = loadModel("models/target.obj", Vec3f(0,1.5f,0));
	//shotTrans = loadModel("models/slingshot.obj");
	projectileTrans = loadModel("models/projectile.obj", Vec3f(0.0f, 0.0f, -2.0f), 1.0f);
	
	
	//root->addChild(landTrans);
	//root->addChild(standTrans);
	//root->addChild(targetTrans);
	//root->addChild(shotTrans);
	root->addChild(projectileTrans);

			
	PointLightRecPtr sunLight = PointLight::create();
	//sunLight->setAttenuation(1,0,2);

	//color information
	sunLight->setDiffuse(Color4f(1,1,1,1));
	sunLight->setAmbient(Color4f(0.2f,0.2f,0.2f,1));
	sunLight->setSpecular(Color4f(1,1,1,1));

	sunLight->setBeacon(sunChild); //attach to the sun node use this node as position beacon

	root->setCore(sunLight);

	DirectionalLightRecPtr dirLight = DirectionalLight::create();
	dirLight->setDirection(1,1,-1);

	//color information
	dirLight->setDiffuse(Color4f(1,1,1,1));
	dirLight->setAmbient(Color4f(0.2f,0.2f,0.2f,1));
	dirLight->setSpecular(Color4f(1,1,1,1));


	//wrap the root, cause only nodes below the lights will be lit
	NodeRecPtr ueberroot = makeNodeFor(dirLight);
	ueberroot->addChild(root);

	root = ueberroot;

	//Skybox
	//initSkybox();
	//root->addChild(skybox.getNodePtr());
	*/
	//return NodeTransitPtr(root);
	return createScenegraph();
}

#define PULL_LOOSE 0
#define PULL_GOING 1
int pullState = PULL_LOOSE;

#define PROJ_DEAD 0
#define PROJ_FLY 1
#define PROJ_PULL 2
int projState = PROJ_DEAD;

Vec3f projSpeed, projPos, targetPos;

void startProjectile(Vec3f pos, Vec3f dir){
	projPos = pos;
	projSpeed = dir;
	projState = PROJ_FLY;
}

void stopProjectile(Vec3f pos){
	projPos = pos;
	projSpeed = Vec3f(0,0,0);
	projState = PROJ_DEAD;
}

void eventStartPull(){
	if(pullState == PULL_GOING){
		return;
	}
	
	Vec3f d1 = schleuder_position - hand_position;
	Vec3f d2 = schleuder_position - head_position;
	std::cout << "dot: " << d1.dot(d2) << std::endl;
	
	if(d1.length() <= 30.0f){
		pullState = PULL_GOING;
		projState = PROJ_PULL;
	}
}

void eventStopPull(){
	if(pullState == PULL_LOOSE){
		return;
	}
	
	Vec3f d1 = schleuder_position - hand_position;	
	startProjectile(hand_position, d1);
	pullState = PULL_LOOSE;
}

void updateSlinshot(float dt){
	if(pullState == PULL_GOING){
		//update slingshot
	}else if(pullState == PULL_LOOSE){
		
	}
}

void updateProjectile(float dt){
	if(projState == PROJ_DEAD){
		return;
	}	
	
	if(projState == PROJ_PULL){
		projPos = hand_position;
	}
	
	if(projState == PROJ_FLY){	
		float fact = 1.0f;
		Vec3f gravity = Vec3f(0.0f, 98.1f, 0.0f);
	
		//new position
		Vec3f nPos = projPos + (projSpeed * dt * fact) + (gravity * dt) ;
	
		//check if target is hit:
		//the flight direction	
		if (targetPos[3] < projPos[3] && targetPos[3] > nPos[3]){
			Vec3f d = nPos - projPos;
			float t = (targetPos[3] - projPos[3]) / d[3];		
			//calc the intersection point:
			Vec3f inter = projPos + (d*t);
		
			Vec3f d2	= targetPos - inter;
			float targetSize = 100.0f;
			if(d2.length() <= targetSize){
				float score = targetSize - d2.length();
				std::cout << "hit target: " << score << std::endl;
				stopProjectile(inter);
			}	
		}
	
		//check if floor is hit 
		if (nPos[2] <= 0.0f){
			Vec3f d = nPos - projPos;
			float t = (0.0f - (float) projPos[2]) / d[2];		
			//calc the intersection point:
			Vec3f inter = projPos + (d*t);
			std::cout << "hit floor: " << std::endl;
			stopProjectile(inter);
		}
	}
	
	//update position finally	
	ComponentTransformRecPtr pt = dynamic_cast<ComponentTransform*>(projectileTrans->getCore());
	pt->setTranslation(projPos);
}

template<typename T>
T scale_tracker2cm(const T& value)
{
	static const float scale = OSGCSM::convert_length(cfg.getUnits(), 1.f, OSGCSM::CAVEConfig::CAVEUnitCentimeters);
	return value * scale;
}


void VRPN_CALLBACK callback_head_tracker(void* userData, const vrpn_TRACKERCB tracker)
{
	head_orientation = Quaternion(tracker.quat[0], tracker.quat[1], tracker.quat[2], tracker.quat[3]);
	head_position = Vec3f(scale_tracker2cm(Vec3d(tracker.pos)));
}


void VRPN_CALLBACK callback_schleuder_tracker(void* userData, const vrpn_TRACKERCB tracker)
{
	schleuder_orientation = Quaternion(tracker.quat[0], tracker.quat[1], tracker.quat[2], tracker.quat[3]);
	schleuder_position = Vec3f(scale_tracker2cm(Vec3d(tracker.pos)));
}

void VRPN_CALLBACK callback_hand_tracker(void* userData, const vrpn_TRACKERCB tracker)
{
	hand_orientation = Quaternion(tracker.quat[0], tracker.quat[1], tracker.quat[2], tracker.quat[3]);
	hand_position = Vec3f(scale_tracker2cm(Vec3d(tracker.pos)));
}

auto analog_values = Vec3f();
void VRPN_CALLBACK callback_analog(void* userData, const vrpn_ANALOGCB analog)
{
	if (analog.num_channel >= 2){
		analog_values = Vec3f(analog.channel[0], 0, -analog.channel[1]);
	}
}

void VRPN_CALLBACK callback_button(void* userData, const vrpn_BUTTONCB button)
{
	if (button.button == 0 && button.state == 1){
		//TODO: check pos and start pulling if true
		eventStartPull();
	}
	if (button.button == 0 && button.state == 0){
		//TODO: check if pulling and release if true
		eventStopPull();
	}
}

void InitTracker(OSGCSM::CAVEConfig &cfg)
{
	try
	{
		const char* const vrpn_name = "DTrack@localhost";
		tracker = new vrpn_Tracker_Remote(vrpn_name);
		tracker->shutup = true;
		tracker->register_change_handler(NULL, callback_head_tracker, cfg.getSensorIDHead());
		tracker->register_change_handler(NULL, callback_hand_tracker, cfg.getSensorIDController());
		tracker->register_change_handler(NULL, callback_schleuder_tracker, 3);
		button = new vrpn_Button_Remote(vrpn_name);
		button->shutup = true;
		button->register_change_handler(nullptr, callback_button);
		analog = new vrpn_Analog_Remote(vrpn_name);
		analog->shutup = true;
		analog->register_change_handler(NULL, callback_analog);
	}
	catch(const std::exception& e) 
	{
		std::cout << "ERROR: " << e.what() << '\n';
		return;
	}
}

void check_tracker()
{
	tracker->mainloop();
	button->mainloop();
	analog->mainloop();
}

void print_tracker()
{
	std::cout << "Head position: " << head_position << " orientation: " << head_orientation << '\n';
	std::cout << "Schleuder position: " << schleuder_position << " orientation: " << schleuder_orientation << '\n';
	std::cout << "Hand position: " << hand_position << " orientation: " << hand_orientation << '\n';
	std::cout << "Analog: " << analog_values << '\n';
}

void keyboard(unsigned char k, int x, int y)
{
	Real32 ed;
	float fac = 5.0f;
	switch(k)
	{
		case 'r':
			head_orientation = Quaternion(Vec3f(0,1,0),3.141f);
			head_position = Vec3f(0.f, 170.f, 200.f);
		break;
		case 'w':
			head_position += Vec3f(0,0, -1.0f)*fac;
			break;
		case 'a':
			head_position += Vec3f(-1.0f,0,0)*fac;
			break;
		case 's':
			head_position += Vec3f(0,0, 1.0f)*fac;
			break;
		case 'd':
			head_position += Vec3f(1.0f,0,0 )*fac;
			break;
		case 'q':
		case 27: 
			//root->clearChildren();
			cleanup();
			exit(EXIT_SUCCESS);
			break;
		case 'e':
			ed = mgr->getEyeSeparation() * .9f;
			std::cout << "Eye distance: " << ed << '\n';
			mgr->setEyeSeparation(ed);
			break;
		case 'E':
			ed = mgr->getEyeSeparation() * 1.1f;
			std::cout << "Eye distance: " << ed << '\n';
			mgr->setEyeSeparation(ed);
			break;
		case 'h':
			cfg.setFollowHead(!cfg.getFollowHead());
			std::cout << "following head: " << std::boolalpha << cfg.getFollowHead() << '\n';
			break;
		case 'i':
			print_tracker();
			break;
		default:
			std::cout << "Key '" << k << "' ignored\n";
	}
}

//****** Update LOOP *******//
void updateLoop(float dt){
	/*
	ComponentTransformRecPtr st = dynamic_cast<ComponentTransform*>(shotTrans->getCore());
	st->setTranslation(schleuder_position);
	st->setRotation(schleuder_orientation);

	
	updateSlinshot(dt);	
	updateProjectile(dt);	
	*/

}

void setupGLUT(int *argc, char *argv[])
{
	glutInit(argc, argv);
	glutInitDisplayMode(GLUT_RGB  |GLUT_DEPTH | GLUT_DOUBLE);
	glutCreateWindow("OpenSG CSMDemo with VRPN API");
	glutDisplayFunc([]()
	{
		// black navigation window
		glClear(GL_COLOR_BUFFER_BIT);
		glutSwapBuffers();
	});
	glutReshapeFunc([](int w, int h)
	{
		mgr->resize(w, h);
		glutPostRedisplay();
	});
	glutKeyboardFunc(keyboard);

	glutMouseFunc([](int button, int state, int x, int y) {
		
		float ang = 0.5f * ((float) (x-150));
		if(state){
			head_orientation = Quaternion(Vec3f(0,1.f,0), 3.141f + osgDegree2Rad(ang));
			
			//rootTrans->setRotation(Quaternion(Vec3f(0,1,0),osgDegree2Rad(ang)));
			std::cout << "Mouse: " << x << ", "<< y<< " angle: " << 3.141f + osgDegree2Rad(ang) << '\n';
		}
		glutPostRedisplay();
	});
	glutIdleFunc([]()
	{
		check_tracker();
		const auto speed = 1.f;
		mgr->setUserTransform(head_position, head_orientation);
		mgr->setTranslation(mgr->getTranslation() + speed * analog_values);
			
		float dt = 1.0f/30.0f;
		//update
		updateLoop(dt);
		
		
		commitChanges();
		mgr->redraw();
		// the changelist should be cleared - else things could be copied multiple times
		OSG::Thread::getCurrentChangeList()->clear();
	});
}

int main(int argc, char **argv)
{
#if WIN32
	OSG::preloadSharedObject("OSGFileIO");
	OSG::preloadSharedObject("OSGImageFileIO");
#endif
	try
	{
		bool cfgIsSet = false;
		NodeRefPtr scene = nullptr;

		// ChangeList needs to be set for OpenSG 1.4
		ChangeList::setReadWriteDefault();
		osgInit(argc,argv);

		// evaluate intial params
		for(int a=1 ; a<argc ; ++a)
		{
			if( argv[a][0] == '-' )
			{
				if ( strcmp(argv[a],"-f") == 0 ) 
				{
					char* cfgFile = argv[a][2] ? &argv[a][2] : &argv[++a][0];
					if (!cfg.loadFile(cfgFile)) 
					{
						std::cout << "ERROR: could not load config file '" << cfgFile << "'\n";
						return EXIT_FAILURE;
					}
					cfgIsSet = true;
				}
			} else {
				std::cout << "Loading scene file '" << argv[a] << "'\n";
				scene = SceneFileHandler::the()->read(argv[a], NULL);
			}
		}

		// load the CAVE setup config file if it was not loaded already:
		if (!cfgIsSet) 
		{
			const char* const default_config_filename = "config/mono.csm";
			if (!cfg.loadFile(default_config_filename)) 
			{
				std::cout << "ERROR: could not load default config file '" << default_config_filename << "'\n";
				return EXIT_FAILURE;
			}
		}

		cfg.printConfig();

		// start servers for video rendering
		if ( startServers(cfg) < 0 ) 
		{
			std::cout << "ERROR: Failed to start servers\n";
			return EXIT_FAILURE;
		}

		setupGLUT(&argc, argv);

		InitTracker(cfg);

		MultiDisplayWindowRefPtr mwin = createAppWindow(cfg, cfg.getBroadcastaddress());

		if (!scene) 
			scene = buildScene();
		commitChanges();

		mgr = new OSGCSM::CAVESceneManager(&cfg);
		mgr->setWindow(mwin );
		mgr->setRoot(scene);
		mgr->setHeadlight(false);
		mgr->showAll();
		mgr->getWindow()->init();
		mgr->turnWandOff();
	}
	catch(const std::exception& e)
	{
		std::cout << "ERROR: " << e.what() << '\n';
		return EXIT_FAILURE;
	}

	glutMainLoop();
}
